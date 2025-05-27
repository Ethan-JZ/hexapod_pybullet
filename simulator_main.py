import pybullet as p
import time
import numpy as np
from pybullet_setup import pybullet_setup
from robot_walk_sim import HexapodRobotWalk
from robot_pose_sim import HexapodRobotPose

#######################
# joint and its index #
# 0  joint11
# 1  joint12
# 2  joint13
# 3  joint41
# 4  joint42
# 5  joint43
# 6  joint21
# 7  joint22
# 8  joint23
# 9  joint51
# 10 joint52
# 11 joint53
# 12 joint61
# 13 joint62
# 14 joint63
# 15 joint31
# 16 joint32
# 17 joint33

if __name__ == "__main__":
    
    # load robot urdf and setup pybullet
    robot_id = pybullet_setup(urdf_path="hexapod_pkg/urdf/hexapod.urdf")

    # init a robot model and start the init pose setup
    hexapod = HexapodRobotPose(robot_id=robot_id)
    hexapod._set_certain_pose(pose_name="standing", duration=0.3)  # Execute smooth movement sequence
    hexapod_walk = HexapodRobotWalk(robot_id=robot_id)

    # paras_turn = {
    #     "gait": "Tripod",  # gait type
    #     "j1_swing": round(np.deg2rad(20), 2), 
    #     "j2_swing": round(np.deg2rad(5), 2),  
    #     "j3_swing": round(np.deg2rad(0), 2),
    # }
    # num_loops = 10  # Number of walking cycles
    # for i in range(num_loops):
    #     hexapod_walk.generate_turning_seq_fk(paras_turn)
    #     hexapod_walk.turn(time_duration=0.2) 

    paras_walk = {
        "gait": "Tripod",  # gait type
        "j1_swing": round(np.deg2rad(20), 2), 
        "j2_swing": round(np.deg2rad(5), 2),  
        "j3_swing": round(np.deg2rad(15), 2),
    }

    num_loops = 30  # Number of walking cycles
    for i in range(num_loops):
        hexapod_walk.generate_walking_seq_fk(paras_walk)
        hexapod_walk.walk(time_duration=0.1)


# Keep simulation running
t = 0
while t < 100:
    p.stepSimulation()
    time.sleep(1./240.)
    t += 1
