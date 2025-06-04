import numpy as np
from helpers.robot_pose_sim import HexapodRobotPose
from helpers.helper import switch_joint_name_to_index
import json


class HexapodRobotWalk(HexapodRobotPose):

    def __init__(self, robot_id):
        super().__init__(robot_id)
        self.current_pose = self.stand_pose_target  # Initialize current pose to lying pose
        self.tripod_group_1 = ["MR", "FL", "BL"]
        self.tripod_group_2 = ["FR", "ML", "BR"]

    def _symmetric_motion(self, initial: float, swing: float):
        """
        Creates a symmetric motion pattern centered around the initial value.
        
        Input:
        initial: float, the initial position of the joint.
        swing: float, the amount of swing to apply to the joint.
        Output:
        motion: np.ndarray, a symmetric motion pattern that starts and ends at the initial position.
        """
        motion = np.array([0, swing]) + initial
        return np.hstack([motion, motion[::-1]])
    
    def move_leg(self, leg: str, 
                 j1_swing: float=float(np.deg2rad(20)), 
                 j2_swing: float=float(np.deg2rad(30)), 
                 j3_swing: float=float(np.deg2rad(20))):
        """
        Move the specified leg in a swinging motion.
        Input:
        leg: str, the leg to move, e.g. "FL", "ML", "BL", "FR", "MR", "BR".
        joint1_swing: float, the swing angle for joint 1 (in radians).
        joint2_swing: float, the swing angle for joint 2 (in radians).
        joint3_swing: float, the swing angle for joint 3 (in radians).
        step_num: int, number of steps in the swinging motion.
        """

        # Get the current joint angles for the specified leg
        theta1_init = self.current_pose[leg][0]
        theta2_init = self.current_pose[leg][1]
        theta3_init = self.current_pose[leg][2]

        if leg not in self.legs:
            raise ValueError(f"Leg {leg} is not a valid leg. Choose from {list(self.legs.keys())}.")
        
        if leg in ["FL", "ML", "BL", "FR", "MR", "BR"]:
            # Calculate the target angles for the leg
            theta1_motion_1 = np.array([0, -j1_swing]) + theta1_init
            theta1_motion_2 = np.array([0, j1_swing]) + theta1_init
            theta1_motion = np.hstack([theta1_motion_1, theta1_motion_2])
            print(f"FL theta1_motion: {theta1_motion}")
            
            theta2_motion = self._symmetric_motion(theta2_init, j2_swing)
            print(f"FL theta2_motion: {theta2_motion}")

            theta3_motion = self._symmetric_motion(theta3_init, j3_swing)
            print(f"FL theta3_motion: {theta3_motion}")

            # Set the target pose for the leg
            for i in range(len(theta1_motion)):
                self._set_smooth_joint_positions(
                    joint_indices=self.legs[leg],
                    target_positions=[theta1_motion[i], theta2_motion[i], theta3_motion[i]],
                    duration=0.5
                )
    
    def generate_turning_seq_fk(self, direction: str, paras:dict) -> None:
        """
        generate a turning sequence for the robot based on the given parameters.
        Input:
        direction: str, the direction to turn, either "clockwise" or "counterclockwise".
        paras: dict, a dictionary containing the parameters for the turning sequence.
               - gait: str, the type of gait to use (e.g., "Tripod").
               - j1_swing: float, the swing angle for joint 1 (in radians).
               - j2_swing: float, the swing angle for joint 2 (in radians).
               - j3_swing: float, the swing angle for joint 3 (in radians).
        Output:
        self.turning_sequence_fk: dict, a dictionary containing the turning sequence for each leg.
                                  The keys are the leg names (e.g., "FL", "ML", "BL", "FR", "MR", "BR"),
                                  and the values are dictionaries with joint angles for each joint.
        """
        # Check if the direction is valid
        if direction not in ["clockwise", "counterclockwise"]:
            raise ValueError("Direction must be either 'clockwise' or 'counterclockwise'.")

        # Extract parameters
        gait     = paras["gait"]
        j1_swing = paras["j1_swing"]
        j2_swing = paras["j2_swing"]
        j3_swing = paras["j3_swing"]

        # Generate the turning sequence based on the gait
        self.turning_sequence_fk = {}

        if gait == "Tripod":
            
            if direction == "counterclockwise":
                for leg in self.legs.keys():
                    # get the current pose of the leg
                    # note that the tipple index is excluded here
                    # self.current_pose[leg][0] is the leg's 1st joint pose: joint_x1
                    # self.current_pose[leg][1] is the leg's 2nd joint pose: joint_x2
                    # self.current_pose[leg][2] is the leg's 3rd joint pose: joint_x3
                    theta1_init = self.current_pose[leg][0]
                    theta2_init = self.current_pose[leg][1]
                    theta3_init = self.current_pose[leg][2]
                    
                    # leg1 in swing state
                    theta1_motion_1 = np.array([0, -j1_swing]) + theta1_init
                    theta1_motion_2 = np.array([0, j1_swing])  + theta1_init
                    theta1_motion = np.hstack([theta1_motion_1, theta1_motion_2])
                    
                    theta2_motion =  self._symmetric_motion(theta2_init, j2_swing) # leg2 in swing state
                    theta3_motion =  self._symmetric_motion(theta3_init, j3_swing) # leg3 in swing state

                    # leg in stance state
                    theta2_stance = theta2_motion[0] * np.ones(int(len(theta2_motion)))  # stance position
                    theta3_stance = theta3_motion[0] * np.ones(int(len(theta3_motion)))  # stance position
                    theta1_stance = theta1_motion[::-1]
                    theta1_stance_a = np.hstack([theta1_motion, theta1_stance])
                    theta1_stance_b = np.hstack([theta1_stance, theta1_motion])

                    # generate the walking sequence
                    if leg in ["MR", "FL", "BL"]:
                        self.turning_sequence_fk[leg] = {
                            "theta1": theta1_stance_a,
                            "theta2": np.hstack([theta2_motion, theta2_stance]),
                            "theta3": np.hstack([theta3_motion, theta3_stance])
                        }
                    else:
                        self.turning_sequence_fk[leg] = {
                            "theta1": theta1_stance_b,
                            "theta2": np.hstack([theta2_stance, theta2_motion]),
                            "theta3": np.hstack([theta3_stance, theta3_motion])
                        }
            else:  # clockwise
                for leg in self.legs.keys():

                    # get the current pose of the leg
                    theta1_init = self.current_pose[leg][0]
                    theta2_init = self.current_pose[leg][1]
                    theta3_init = self.current_pose[leg][2]

                    # leg1 in swing state
                    theta1_motion_1 = np.array([0, j1_swing]) + theta1_init
                    theta1_motion_2 = np.array([0, -j1_swing])  + theta1_init
                    theta1_motion = np.hstack([theta1_motion_1, theta1_motion_2])
                    
                    theta2_motion =  self._symmetric_motion(theta2_init, j2_swing) # leg2 in swing state
                    theta3_motion =  self._symmetric_motion(theta3_init, j3_swing) # leg3 in swing state

                    # leg in stance state
                    theta2_stance = theta2_motion[0] * np.ones(int(len(theta2_motion)))  # stance position
                    theta3_stance = theta3_motion[0] * np.ones(int(len(theta3_motion)))  # stance position
                    theta1_stance = theta1_motion[::-1]
                    theta1_stance_a = np.hstack([theta1_motion, theta1_stance])
                    theta1_stance_b = np.hstack([theta1_stance, theta1_motion])

                    # generate the walking sequence
                    if leg in self.tripod_group_1:
                        self.turning_sequence_fk[leg] = {
                            "theta1": theta1_stance_a,
                            "theta2": np.hstack([theta2_motion, theta2_stance]),
                            "theta3": np.hstack([theta3_motion, theta3_stance])
                        }
                    else:
                        self.turning_sequence_fk[leg] = {
                            "theta1": theta1_stance_b,
                            "theta2": np.hstack([theta2_stance, theta2_motion]),
                            "theta3": np.hstack([theta3_stance, theta3_motion])
                        }
        
        # the result self.turning_sequence_fk:
        # {"FL": {"theta1": [], "theta2": [], "theta3": []},
        #  "FR": {"theta1": [], "theta2": [], "theta3": []},
        #  "ML": {"theta1": [], "theta2": [], "theta3": []},
        #  "MR": {"theta1": [], "theta2": [], "theta3": []},
        #  "BL": {"theta1": [], "theta2": [], "theta3": []},
        #  "BR": {"theta1": [], "theta2": [], "theta3": []},}
        
        return self.turning_sequence_fk

    def generate_walking_seq_fk(self, paras:dict):
        """
        generate a walking sequence for the robot based on the given parameters.
        Input:
        paras: dict, a dictionary containing the parameters for the walking sequence.
               - gait: str, the type of gait to use (e.g., "Tripod").
               - j1_swing: float, the swing angle for joint 1 (in radians).
               - j2_swing: float, the swing angle for joint 2 (in radians).
               - j3_swing: float, the swing angle for joint 3 (in radians).
        Output:
        self.walking_sequence_fk: dict, a dictionary containing the walking sequence for each leg.
                                  The keys are the leg names (e.g., "FL", "ML", "BL", "FR", "MR", "BR"),
                                  and the values are dictionaries with joint angles for each joint.
        """

        # Extract parameters
        gait     = paras["gait"]
        j1_swing = paras["j1_swing"]
        j2_swing = paras["j2_swing"]
        j3_swing = paras["j3_swing"]

        # Generate the walking sequence based on the gait
        self.walking_sequence_fk = {}

        if gait == "Tripod":
            for leg in self.legs.keys():
                # get the current pose of the leg
                # note that the tipple index is excluded here
                # self.current_pose[leg][0] is the leg's 1st joint pose: joint_x1
                # self.current_pose[leg][1] is the leg's 2nd joint pose: joint_x2
                # self.current_pose[leg][2] is the leg's 3rd joint pose: joint_x3
                theta1_init = self.current_pose[leg][0]
                theta2_init = self.current_pose[leg][1]
                theta3_init = self.current_pose[leg][2]

                # leg in swing state
                # determine the direction of the swing of joint 1
                if leg in self.tripod_group_2:
                    theta1_motion_1 = np.array([0, -j1_swing]) + theta1_init
                    theta1_motion_2 = np.array([0, j1_swing])  + theta1_init
                    theta1_motion = np.hstack([theta1_motion_1, theta1_motion_2])
                else:
                    theta1_motion_1 = np.array([0, j1_swing]) + theta1_init
                    theta1_motion_2 = np.array([0, -j1_swing]) + theta1_init
                    theta1_motion = np.hstack([theta1_motion_1, theta1_motion_2])
                
                theta2_motion =  self._symmetric_motion(theta2_init, j2_swing) # leg2 in swing state
                theta3_motion =  self._symmetric_motion(theta3_init, j3_swing) # leg3 in swing state

                # leg in stance state
                theta2_stance = theta2_motion[0] * np.ones(int(len(theta2_motion)))  # stance position
                theta3_stance = theta3_motion[0] * np.ones(int(len(theta3_motion)))  # stance position
                theta1_stance = theta1_motion[::-1]
                theta1_stance_a = np.hstack([theta1_motion, theta1_stance])
                theta1_stance_b = np.hstack([theta1_stance, theta1_motion])

                # generate the walking sequence
                if leg in self.tripod_group_1:
                    self.walking_sequence_fk[leg] = {
                        "theta1": theta1_stance_a,
                        "theta2": np.hstack([theta2_motion, theta2_stance]),
                        "theta3": np.hstack([theta3_motion, theta3_stance])
                    }
                else:
                    self.walking_sequence_fk[leg] = {
                        "theta1": theta1_stance_b,
                        "theta2": np.hstack([theta2_stance, theta2_motion]),
                        "theta3": np.hstack([theta3_stance, theta3_motion])
                    }
        
        # the result self.walking_sequence_fk:
        # {"FL": {"theta1": [], "theta2": [], "theta3": []},
        #  "FR": {"theta1": [], "theta2": [], "theta3": []},
        #  "ML": {"theta1": [], "theta2": [], "theta3": []},
        #  "MR": {"theta1": [], "theta2": [], "theta3": []},
        #  "BL": {"theta1": [], "theta2": [], "theta3": []},
        #  "BR": {"theta1": [], "theta2": [], "theta3": []},}

        return self.walking_sequence_fk
    
    def _reformat_to_joint_sequence(self, seq: dict) -> dict:
        # Create mapping from leg names to numbers
        leg_mapping = {
            'FL': 1,
            'ML': 2,
            'BL': 3,
            'FR': 4,
            'MR': 5,
            'BR': 6
        }

        joint_wise_sequence = {}
        
        # loop each leg, like "FL"
        for leg_name, leg_data in seq.items():
            leg_number = leg_mapping[leg_name]  # e.g. FL is 1

            # make sure the leg_data is a dictionary
            if not isinstance(leg_data, dict):
                raise ValueError(f"Expected leg data for {leg_name} to be a dictionary, got {type(leg_data)} instead.")
            
            # loop each leg's {"theta1": [], "theta2": [], "theta3": []}
            for joint_name, joint_data in leg_data.items():
                
                # extract joint number from the joint name, e.g. theta1 -> 1
                joint_number = int(joint_name[-1])

                # form the new key: e.g. joint_11
                new_key = f"joint_{leg_number}{joint_number}"
                joint_wise_sequence[new_key] = joint_data
                
        # this will return: {"joint_11": [], "joint_12": [], "joint_13": [], ...}
        return joint_wise_sequence
    
    def turn(self, time_duration: float=0.5) -> None:
        """turn the robot by executing the turning sequence.
        Input:
        direction: str, the direction to turn, either "clockwise" or "counterclockwise".
        time_duration: float, the duration of each step in seconds.
        Output:
        None, the robot will execute the turning sequence.
        """
        # Check if the turning sequence is generated
        if not hasattr(self, 'turning_sequence_fk'):
            raise ValueError("Turning sequence not generated. Call generate_turning_sequence_fk() first.")
        
        # get the length of the turning sequence
        sequence_length = len(self.turning_sequence_fk["FL"]["theta1"])

        # reformat the turning sequence data
        joint_wise_seqences = self._reformat_to_joint_sequence(self.turning_sequence_fk)

        # get all joint indices for the legs
        joint_indices = []
        joint_target_positions = []

        for joint_name in joint_wise_seqences.keys():
            # switch joint name to index
            joint_index = switch_joint_name_to_index(self.robot_id, joint_name)
            joint_indices.append(joint_index)

        # get the target positions for each joint in the sequence, this will be a list of lists
        # e.g. joint_wise_seqences = {"joint11": [0.1, 0.2, ...], "joint12": [0.3, 0.4, ...], ...}
        # and joint_target_positions = [[0.1, 0.3, ...], [0.2, 0.4, ...], ...]
        joint_target_positions = [list(values) for values in zip(*joint_wise_seqences.values())]
        
        for i in range(sequence_length):
            self._set_smooth_joint_positions(joint_indices, joint_target_positions[i], duration=time_duration)

    def walk(self, time_duration: float=0.5) -> None:
        """walk the robot by executing the walking sequence.
        Input:
        time_duration: float, the duration of each step in seconds.
        Output:
        None, the robot will execute the walking sequence.
        """
        
        # Check if the walking sequence is generated
        if not hasattr(self, 'walking_sequence_fk'):
            raise ValueError("Walking sequence not generated. Call generate_walking_sequence_fk() first.")
        
        # get the length of the walking sequence
        sequence_length = len(self.walking_sequence_fk["FL"]["theta1"])

        # reformat the walking sequence data
        joint_wise_seqences = self._reformat_to_joint_sequence(self.walking_sequence_fk)

        # get all joint indices for the legs
        joint_indices = []
        joint_target_positions = []

        for joint_name in joint_wise_seqences.keys():
            # switch joint name to index
            joint_index = switch_joint_name_to_index(self.robot_id, joint_name)
            joint_indices.append(joint_index)

        # get the target positions for each joint in the sequence, this will be a list of lists
        # e.g. joint_wise_seqences = {"joint11": [0.1, 0.2, ...], "joint12": [0.3, 0.4, ...], ...}
        # and joint_target_positions = [[0.1, 0.3, ...], [0.2, 0.4, ...], ...]
        joint_target_positions = [list(values) for values in zip(*joint_wise_seqences.values())]

        for i in range(sequence_length):
            self._set_smooth_joint_positions(joint_indices, joint_target_positions[i], duration=time_duration)
            