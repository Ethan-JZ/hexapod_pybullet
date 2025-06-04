import pybullet as p
import json
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
        "j1_swing": round(np.deg2rad(10), 2), 
        "j2_swing": round(np.deg2rad(5), 2),  
        "j3_swing": round(np.deg2rad(10), 2),
    }

    walking_seq = {
        "joint_11": [], "joint_12": [], "joint_13": [],
        "joint_41": [], "joint_42": [], "joint_43": [],
        "joint_21": [], "joint_22": [], "joint_23": [],
        "joint_51": [], "joint_52": [], "joint_53": [],
        "joint_61": [], "joint_62": [], "joint_63": [],
        "joint_31": [], "joint_32": [], "joint_33": []
    }

    num_loops = 100  # Number of walking cycles
    for i in range(num_loops):
        walking_seq_return = hexapod_walk.generate_walking_seq_fk(paras=paras_walk)
        for leg_name in walking_seq.keys():
            if leg_name == "FL":
                walking_seq["joint_11"].extend(walking_seq_return[leg_name]['theta1'])
                walking_seq["joint_12"].extend(walking_seq_return[leg_name]['theta2'])
                walking_seq["joint_13"].extend(walking_seq_return[leg_name]['theta3'])
            elif leg_name == "FR":
                walking_seq["joint_41"].extend(walking_seq_return[leg_name]['theta1'])
                walking_seq["joint_42"].extend(walking_seq_return[leg_name]['theta2'])
                walking_seq["joint_43"].extend(walking_seq_return[leg_name]['theta3'])
            elif leg_name == "ML":
                walking_seq["joint_21"].extend(walking_seq_return[leg_name]['theta1'])
                walking_seq["joint_22"].extend(walking_seq_return[leg_name]['theta2'])
                walking_seq["joint_23"].extend(walking_seq_return[leg_name]['theta3'])
            elif leg_name == "MR":
                walking_seq["joint_51"].extend(walking_seq_return[leg_name]['theta1'])
                walking_seq["joint_52"].extend(walking_seq_return[leg_name]['theta2'])
                walking_seq["joint_53"].extend(walking_seq_return[leg_name]['theta3'])
            elif leg_name == "BL":
                walking_seq["joint_31"].extend(walking_seq_return[leg_name]['theta1'])
                walking_seq["joint_32"].extend(walking_seq_return[leg_name]['theta2'])
                walking_seq["joint_33"].extend(walking_seq_return[leg_name]['theta3'])
            elif leg_name == "BR":
                walking_seq["joint_61"].extend(walking_seq_return[leg_name]['theta1'])
                walking_seq["joint_62"].extend(walking_seq_return[leg_name]['theta2'])
                walking_seq["joint_63"].extend(walking_seq_return[leg_name]['theta3'])

        hexapod_walk.walk(time_duration=0.1)
    
    # save the turning sequence to a JSON file
    file_path="helpers/data_cache/walking_seq.json"
    with open(file_path, 'w') as f:
        json.dump(walking_seq, f, indent=4)
