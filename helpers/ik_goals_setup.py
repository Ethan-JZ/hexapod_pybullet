from robot_pose_sim import HexapodRobotPose

class IKGoalsSetup:

    def __init__(self, robot_id):

        self.robot_id = robot_id
        self.robot_pose_obj = HexapodRobotPose(self.robot_id)
    
    def _get_default_standing_positions(self):
        joint1 = self.robot_pose_obj.joint1_init
        joint2 = self.robot_pose_obj.joint2_standing
        joint3 = self.robot_pose_obj.joint3_standing
