import pybullet as p
import numpy as np
import time


class HexapodRobotPose:

    def __init__(self, robot_id):

        # initialize the robot id and control group
        self.robot_id = robot_id
        self.legs = self._define_control_group()
        
        # set control parameters
        self.control_mode = p.POSITION_CONTROL
        self.max_angular_v = 1.0
        self.kp = 0.5
        self.kd = 0.1
        
        # define init pose target 
        joint1_init     = [np.pi / 6, 0, -np.pi / 6, -np.pi / 6, 0, np.pi / 6]
        joint2_init     = [np.deg2rad(0)] * 6
        joint3_init     = [np.deg2rad(0)] * 6
        joint2_feet_up  = [np.deg2rad(90)] * 6
        joint3_feet_up  = [-np.deg2rad(60)] * 6
        joint2_standing = [np.deg2rad(20)] * 6
        joint3_standing = [-np.deg2rad(60)] * 6
        
        self.lying_pose_target = self._pose_target_in_leg(joint1_init, joint2_init, joint3_init)
        self.stand_pose_target = self._pose_target_in_leg(joint1_init, joint2_standing, joint3_standing)
        self.feet_up_pose_target = self._pose_target_in_leg(joint1_init, joint2_feet_up, joint3_feet_up)
        
    def _pose_target_in_leg(self, j1_target: list, j2_target: list, j3_target: list) -> dict:

        # round the joint angles to 2 decimal places
        j1_target = np.around(j1_target, 2)
        j2_target = np.around(j2_target, 2)
        j3_target = np.around(j3_target, 2)

        pose = {
            "FL": [j1_target[0], j2_target[0], j3_target[0]], 
            "ML": [j1_target[1], j2_target[1], j3_target[1]], 
            "BL": [j1_target[2], j2_target[2], j3_target[2]], 
            "FR": [j1_target[3], j2_target[3], j3_target[3]], 
            "MR": [j1_target[4], j2_target[4], j3_target[4]], 
            "BR": [j1_target[5], j2_target[5], j3_target[5]]
            }

        return pose
    
    def _define_control_group(self) -> dict:
        """
        Define the control group for each leg based on joint indices.
        
        Input:
        None
        Output:
        legs: dict, a dictionary containing the joint indices for each leg
              e.g. {"FL": [0, 1, 2], "ML": [3, 4, 5], ...}
        """
        # init control group 
        legs = {
            "FL": [], 
            "ML": [], 
            "BL": [], 
            "FR": [], 
            "MR": [],
            "BR": []
            }
        
        num_joints = p.getNumJoints(self.robot_id)

        # Loop through all joints and assign them to different groups
        for joint_idx in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            joint_str = joint_info[1].decode("utf-8")

            if joint_str[-2] == "1":
                legs["FL"].append(joint_idx)
            elif joint_str[-2] == "2":
                legs["ML"].append(joint_idx)
            elif joint_str[-2] == "3":
                legs["BL"].append(joint_idx)
            elif joint_str[-2] == "4":
                legs["FR"].append(joint_idx)
            elif joint_str[-2] == "5":
                legs["MR"].append(joint_idx)
            else:
                legs["BR"].append(joint_idx)

        return legs
    
    def _set_smooth_joint_positions(self, joint_indices: list, target_positions: list, duration: float=0.5) -> None:
        """
        smoothly move joints to target positions over given duration
        
        Input:
        joint_indices: list, indices of the joints to control
        target_positions: list, target positions for the joints
        duration: float, time in seconds to reach the target positions
        Output:
        None
        """
        steps = int(duration * 240)  # 240 Hz simulation
        
        # handling the small total steps issue 
        if steps < 1:
            steps = 1
        
        # get all current joint positions
        current_positions = [p.getJointState(self.robot_id, j)[0] for j in joint_indices]
        
        # move the robot step by step
        for step in range(steps):
            alpha = step / steps
            interp_positions = [
                current + alpha * (target - current)
                for current, target in zip(current_positions, target_positions)
            ]
            
            p.setJointMotorControlArray(
                self.robot_id,
                joint_indices,
                self.control_mode,
                targetPositions=interp_positions,
                forces=[self.max_angular_v]*len(joint_indices),
                positionGains=[self.kp]*len(joint_indices),
                velocityGains=[self.kd]*len(joint_indices)
            )
            
            p.stepSimulation()
            time.sleep(1./240.)

    def _set_certain_pose(self, pose_name: str, duration: float=1.0) -> dict:
        """
        Set the robot to a certain pose based on the pose name.
        Input:
        pose_name: str, name of the pose to set (e.g., "lying", "feet_up", "standing")
        duration: float, time in seconds to reach the target pose
        Output:
        target_pose: dict, the target pose for each leg
        """

        # move each leg with the target pose
        leg_FL_indices = self.legs["FL"]
        leg_ML_indices = self.legs["ML"]
        leg_BL_indices = self.legs["BL"]
        leg_FR_indices = self.legs["FR"]
        leg_MR_indices = self.legs["MR"]
        leg_BR_indices = self.legs["BR"]
        

        if pose_name == "lying":
            target_pose = self.lying_pose_target
        elif pose_name == "feet_up":
            target_pose = self.feet_up_pose_target
        elif pose_name == "standing":
            target_pose = self.stand_pose_target

        self._set_smooth_joint_positions(leg_FL_indices, target_pose["FL"], duration) # control leg FL
        self._set_smooth_joint_positions(leg_ML_indices, target_pose["ML"], duration) # control leg ML
        self._set_smooth_joint_positions(leg_BL_indices, target_pose["BL"], duration) # control leg BL
        self._set_smooth_joint_positions(leg_FR_indices, target_pose["FR"], duration) # control leg FR
        self._set_smooth_joint_positions(leg_MR_indices, target_pose["MR"], duration) # control leg MR
        self._set_smooth_joint_positions(leg_BR_indices, target_pose["BR"], duration) # control leg BR

        return target_pose

    def move_from_lying_to_standing(self, transition_duration: float=1.0, pause_time: float=0.5):

        """
        Move the robot from lying pose to standing pose through a sequence of poses.
        Input:
        transition_duration: float, time in seconds to transition between poses
        pause_time: float, time in seconds to pause at each pose
        Output:
        None
        """
        
        # start at lying pose
        _ = self._set_certain_pose(pose_name="lying", duration=transition_duration)
        time.sleep(pause_time)

        # to foot-up pose
        _ = self._set_certain_pose(pose_name="feet_up", duration=transition_duration)
        time.sleep(pause_time)

        # to standing pose
        _ = self._set_certain_pose(pose_name="standing", duration=transition_duration)
        time.sleep(3)
    