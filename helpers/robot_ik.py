import numpy as np
import pybullet as p

#######################
# joint and its index #
# 0   joint_11
# 1   joint_12
# 2   joint_13
# 3   joint_1_tipple
# 4   joint_41
# 5   joint_42
# 6   joint_43
# 7   joint_4_tipple
# 8   joint_21
# 9   joint_22
# 10  joint_23
# 11  joint_2_tipple
# 12  joint_51
# 13  joint_52
# 14  joint_53
# 15  joint_5_tipple
# 16  joint_61
# 17  joint_62
# 18  joint_63
# 19  joint_6_tipple
# 20  joint_31
# 21  joint_32
# 22  joint_33
# 23  joint_3_tipple


class RobotIKSolver:

    def __init__(self, tipple_goals: list, robot_id: int):
        self.tipple_goals = tipple_goals  # [[1x6], [1x6], [1x6], [1x6], [1x6], [1x6]]
        self.robot_id     = robot_id  # an integer
        self.rest_poses   = [0, 0.2, -0.2]

        self.moving_seq   = self._solve_ik_all()
    
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
    
    def _solve_ik_all(self):
        """
        Solve ik for all legs
        """

        # initialize the leg moving sequence
        self.moving_seq = {"leg1": {"theta1": [], "theta2": [], "theta3": []},
                           "leg4": {"theta1": [], "theta2": [], "theta3": []},
                           "leg2": {"theta1": [], "theta2": [], "theta3": []},
                           "leg5": {"theta1": [], "theta2": [], "theta3": []},
                           "leg6": {"theta1": [], "theta2": [], "theta3": []},
                           "leg3": {"theta1": [], "theta2": [], "theta3": []}}
        
        # loop all goal from the tipple goals
        for tipple_goal in self.tipple_goals:
            # tipple_goal = [goal1, goal4, goal2, goal5, goal6, goal3], numbers are for legs

            # compute joint poses for each leg
            leg1_joint_poses = self._solve_ik_leg(joint_start_index=0, 
                                                joint_end_index=2, target_position=tipple_goal[0])
            leg4_joint_poses = self._solve_ik_leg(joint_start_index=4, 
                                                joint_end_index=6, target_position=tipple_goal[1])
            
            leg2_joint_poses = self._solve_ik_leg(joint_start_index=8, 
                                                joint_end_index=10, target_position=tipple_goal[2])
            leg5_joint_poses = self._solve_ik_leg(joint_start_index=12, 
                                                joint_end_index=14, target_position=tipple_goal[3])
            
            leg6_joint_poses = self._solve_ik_leg(joint_start_index=16, 
                                                joint_end_index=18, target_position=tipple_goal[4])
            leg3_joint_poses = self._solve_ik_leg(joint_start_index=20, 
                                                joint_end_index=22, target_position=tipple_goal[5])
            
            # pack the data
            self.moving_seq["leg1"]["theta1"].append(leg1_joint_poses[0])
            self.moving_seq["leg1"]["theta2"].append(leg1_joint_poses[1])
            self.moving_seq["leg1"]["theta3"].append(leg1_joint_poses[2])

            self.moving_seq["leg4"]["theta1"].append(leg4_joint_poses[0])
            self.moving_seq["leg4"]["theta2"].append(leg4_joint_poses[1])
            self.moving_seq["leg4"]["theta3"].append(leg4_joint_poses[2])

            self.moving_seq["leg2"]["theta1"].append(leg2_joint_poses[0])
            self.moving_seq["leg2"]["theta2"].append(leg2_joint_poses[1])
            self.moving_seq["leg2"]["theta3"].append(leg2_joint_poses[2])

            self.moving_seq["leg5"]["theta1"].append(leg5_joint_poses[0])
            self.moving_seq["leg5"]["theta2"].append(leg5_joint_poses[1])
            self.moving_seq["leg5"]["theta3"].append(leg5_joint_poses[2])

            self.moving_seq["leg6"]["theta1"].append(leg6_joint_poses[0])
            self.moving_seq["leg6"]["theta2"].append(leg6_joint_poses[1])
            self.moving_seq["leg6"]["theta3"].append(leg6_joint_poses[2])

            self.moving_seq["leg3"]["theta1"].append(leg3_joint_poses[0])
            self.moving_seq["leg3"]["theta2"].append(leg3_joint_poses[1])
            self.moving_seq["leg3"]["theta3"].append(leg3_joint_poses[2])
        
        return self.moving_seq

