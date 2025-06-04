import numpy as np
import pybullet as p


class RobotIKSolver:

    def __init__(self, tipple_goal, robot_id):
        self.tipple_goal = tipple_goal
        self.robot_id = robot_id
        self.rest_poses = [0, 0.2, -0.2]
    
    def _solve_ik_leg(self, joint_start_index: int, joint_end_index: int, target_position=None):
        """
        Compute ik for the leg based on joint start index, which defines the leg
        """
        leg_tipple_index = joint_end_index + 1
        leg_joint_poses = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=leg_tipple_index,
            targetPosition=target_position,
            restPoses=self.rest_poses,
            solver=p.IK_DLS,
            maxNumIterations=100,
            residualThreshold=1e-5
        )
        return leg_joint_poses[joint_start_index:joint_end_index+1]
    
    def _solve_ik_all(self, target_positions: list):
        """
        Solve ik for all legs
        Input:
        target_positions: a list of lists: [[1x3], [1x3], [1x3], [1x3], [1x3], [1x3]]
        target_positions[0]: leg 1 target position
        target_positions[1]: leg 4 target position
        target_positions[2]: leg 2 target position
        target_positions[3]: leg 5 target position
        target_positions[4]: leg 6 target position
        target_positions[5]: leg 3 target position
        """
        leg1_joint_poses = self._solve_ik_leg(joint_start_index=0, 
                                              joint_end_index=2, target_position=target_positions[0])
        leg4_joint_poses = self._solve_ik_leg(joint_start_index=4, 
                                              joint_end_index=2, target_position=target_positions[0])



if __name__ == "__main__":
    robot_ik_model = RobotIKSolver(None, "hexapod_pkg/urdf/hexapod.urdf")
    target_position = [0, 0, 0]
    robot_ik_model.solve_ik(joint_start_index=0, joint_end_index=2, target_position=target_position)