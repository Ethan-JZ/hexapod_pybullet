import pybullet as p
import time
import numpy as np
from helpers.pybullet_setup import pybullet_setup
from helpers.robot_walk_sim import HexapodRobotWalk
from helpers.robot_pose_sim import HexapodRobotPose
from helpers.helper import get_joint_info

#######################
# joint and its index #
# 0  joint_11
# 1  joint_12
# 2  joint_13
# 3  joint_41
# 4  joint_42
# 5  joint_43
# 6  joint_21
# 7  joint_22
# 8  joint_23
# 9  joint_51
# 10 joint_52
# 11 joint_53
# 12 joint_61
# 13 joint_62
# 14 joint_63
# 15 joint_31
# 16 joint_32
# 17 joint_33

if __name__ == "__main__":
    
    # load robot urdf and setup pybullet
    robot_id = pybullet_setup(urdf_path="hexapod_pkg/urdf/hexapod.urdf")
    get_joint_info(robot_id)  # Print joint information for debugging

    # init a robot model and start the init pose setup
    hexapod = HexapodRobotPose(robot_id=robot_id)
    hexapod._set_certain_pose(pose_name="standing", duration=0.2)  # Execute smooth movement sequence
    hexapod_walk = HexapodRobotWalk(robot_id=robot_id)

    paras_walk = {
        "gait": "Tripod",  # gait type
        "j1_swing": round(np.deg2rad(20), 2), 
        "j2_swing": round(np.deg2rad(5), 2),  
        "j3_swing": round(np.deg2rad(10), 2),
    }

    num_loops = 100  # Number of walking cycles
    for i in range(num_loops):
        hexapod_walk.generate_walking_seq_fk(paras_walk)
        hexapod_walk.walk(time_duration=0.1)
