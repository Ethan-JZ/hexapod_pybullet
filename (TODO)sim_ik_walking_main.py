from helpers.robot_ik import RobotIKSolver
from helpers.pybullet_setup import pybullet_setup


if __name__ == "__main__":

    # load robot urdf and setup pybullet
    robot_id = pybullet_setup(urdf_path="hexapod_pkg/urdf/hexapod.urdf")

    # solve the ik based on tipple goals
    tipple_goals = None
    robot_ik_model = RobotIKSolver(tipple_goals=tipple_goals, robot_id=robot_id)
    walking_ik_seq_return = robot_ik_model.moving_seq()