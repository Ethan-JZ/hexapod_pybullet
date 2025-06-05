import pybullet as p
import json
import numpy as np
from helpers.pybullet_setup import pybullet_setup
from helpers.robot_walk_sim import HexapodRobotWalk
from helpers.robot_pose_sim import HexapodRobotPose
from helpers.helper import get_joint_info

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

if __name__ == "__main__":
    
    # load robot urdf and setup pybullet
    robot_id = pybullet_setup(urdf_path="hexapod_pkg/urdf/hexapod.urdf")
    get_joint_info(robot_id)  # Print joint information for debugging

    # init a robot model and start the init pose setup
    hexapod = HexapodRobotPose(robot_id=robot_id)
    hexapod._set_certain_pose(pose_name="standing", duration=0.2)  # Execute smooth movement sequence
    hexapod_walk = HexapodRobotWalk(robot_id=robot_id)

    paras_turn = {
        "gait": "Tripod",  # gait type
        "j1_swing": round(np.deg2rad(20), 2), 
        "j2_swing": round(np.deg2rad(10), 2),  
        "j3_swing": round(np.deg2rad(15), 2),
    }

    turning_seq = {
        "joint_11": [], "joint_12": [], "joint_13": [],
        "joint_41": [], "joint_42": [], "joint_43": [],
        "joint_21": [], "joint_22": [], "joint_23": [],
        "joint_51": [], "joint_52": [], "joint_53": [],
        "joint_61": [], "joint_62": [], "joint_63": [],
        "joint_31": [], "joint_32": [], "joint_33": []
    }
    num_loops = 100  # Number of walking cycles
    for i in range(num_loops):
        turning_seq_clockwise = hexapod_walk.generate_turning_seq_fk(direction="clockwise", paras=paras_turn)
        for leg_name in turning_seq_clockwise.keys():
            if leg_name == "FL":
                turning_seq["joint_11"].extend(turning_seq_clockwise[leg_name]['theta1'])
                turning_seq["joint_12"].extend(turning_seq_clockwise[leg_name]['theta2'])
                turning_seq["joint_13"].extend(turning_seq_clockwise[leg_name]['theta3'])
            elif leg_name == "FR":
                turning_seq["joint_41"].extend(turning_seq_clockwise[leg_name]['theta1'])
                turning_seq["joint_42"].extend(turning_seq_clockwise[leg_name]['theta2'])
                turning_seq["joint_43"].extend(turning_seq_clockwise[leg_name]['theta3'])
            elif leg_name == "ML":
                turning_seq["joint_21"].extend(turning_seq_clockwise[leg_name]['theta1'])
                turning_seq["joint_22"].extend(turning_seq_clockwise[leg_name]['theta2'])
                turning_seq["joint_23"].extend(turning_seq_clockwise[leg_name]['theta3'])
            elif leg_name == "MR":
                turning_seq["joint_51"].extend(turning_seq_clockwise[leg_name]['theta1'])
                turning_seq["joint_52"].extend(turning_seq_clockwise[leg_name]['theta2'])
                turning_seq["joint_53"].extend(turning_seq_clockwise[leg_name]['theta3'])
            elif leg_name == "BL":
                turning_seq["joint_31"].extend(turning_seq_clockwise[leg_name]['theta1'])
                turning_seq["joint_32"].extend(turning_seq_clockwise[leg_name]['theta2'])
                turning_seq["joint_33"].extend(turning_seq_clockwise[leg_name]['theta3'])
            elif leg_name == "BR":
                turning_seq["joint_61"].extend(turning_seq_clockwise[leg_name]['theta1'])
                turning_seq["joint_62"].extend(turning_seq_clockwise[leg_name]['theta2'])
                turning_seq["joint_63"].extend(turning_seq_clockwise[leg_name]['theta3'])
            
        hexapod_walk.turn(time_duration=0.2)
    
    # save the turning sequence to a JSON file
    file_path="helpers/data_cache/turning_seq_clockwise.json"
    with open(file_path, 'w') as f:
        json.dump(turning_seq, f, indent=4)

