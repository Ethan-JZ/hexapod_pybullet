import numpy as np
import pybullet as p


class RobotIKSolver:

    def __init__(self, tipple_goal, robot_id):
        self.tipple_goal = tipple_goal
        self.robot_id = robot_id
        self.rest_poses = [0, 0.2, -0.2]

        self.leg1_indices = [0, 1 ,2]
        self.leg4_indices = [3, 4 ,5]
        self.leg2_indices = [6, 7 ,8]
        self.leg5_indices = [9, 10 ,11]
        self.leg6_indices = [12, 13 ,14]
        self.leg3_indices = [15, 16 ,17]
    
    def _solve_ik(self, leg_tipple_index, target_position=None):
        leg_joint_poses = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=leg_tipple_index,
            targetPosition=target_position,
            restPoses=self.rest_poses,
            solver=p.IK_DLS,
            maxNumIterations=100,
            residualThreshold=1e-5
        )

if __name__ == "__main__":
    robot_ik_model = RobotIKSolver(None, "hexapod_pkg/urdf/hexapod.urdf")
    robot_ik_model._solve_ik()