from helpers.lx16a import *
from helpers.robot_poses import standing_pose, laying_down_pose, feet_up_pose, set_pose, feet_retrieve_pose
from helpers.read_turning_data import MapSimToReal
import time


def set_joint_positions(joints, motor_frame_data):
    """
    Set the joint positions of the robot based on the motor frame data.
    Input:
    joints: dict, a dictionary containing the joint objects
    motor_frame_data: list, a list of target angles for each joint in the motor frame
    Output:
    Moves each joint to the target angle in the specified time.
    """
    for i in range(len(motor_frame_data)):
        time.sleep(0.4)  # wait for a short time before setting the next pose
        set_pose(joints, motor_frame_data[i])

def leg_groups(joints):

    leg_group = {"1": [joints["11"], joints["12"], joints["13"]],
                 "2": [joints["21"], joints["22"], joints["23"]],
                 "3": [joints["31"], joints["32"], joints["33"]],
                 "4": [joints["41"], joints["42"], joints["43"]],
                 "5": [joints["51"], joints["52"], joints["53"]],
                 "6": [joints["61"], joints["62"], joints["63"]]}
    
    return leg_group


def init_robot_motors(port_str: str):

    # init the port
    LX16A.initialize(port_str)

    # left side legs
    servo11, servo12, servo13 = [LX16A(id) for id in [11, 12, 13]]
    servo21, servo22, servo23 = [LX16A(id) for id in [21, 22, 23]]
    servo31, servo32, servo33 = [LX16A(id) for id in [31, 32, 33]]

    # right side legs
    servo41, servo42, servo43 = [LX16A(id) for id in [41, 42, 43]]
    servo51, servo52, servo53 = [LX16A(id) for id in [51, 52, 53]]
    servo61, servo62, servo63 = [LX16A(id) for id in [61, 62, 63]]

    joints = {"11": servo11, "12": servo12, "13": servo13, 
              "21": servo21, "22": servo22, "23": servo23, 
              "31": servo31, "32": servo32, "33": servo33, 
              "41": servo41, "42": servo42, "43": servo43, 
              "51": servo51, "52": servo52, "53": servo53, 
              "61": servo61, "62": servo62, "63": servo63}

    return joints


def init_pose(joints):

    laying_down_pose(joints) # execute laying down pose
    time.sleep(1)

    feet_up_pose(joints) # execute feet up pose
    time.sleep(1)

    standing_pose(joints) # standing pose
    time.sleep(1)
    
    
if __name__ == '__main__':

    # set the port name
    port_str = "COM4"

    # set the joints related to each motor
    joints = init_robot_motors(port_str)

    # init the pose
    init_pose(joints)

    mapping_object   = MapSimToReal(file_path="helpers/data_cache/turning_seq_clockwise.json")
    motor_frame_data = mapping_object.convert_degree_to_motor_frame()
    set_joint_positions(joints, motor_frame_data)
    time.sleep(1)

    laying_down_pose(joints) # execute feet up pose
    time.sleep(1)

    feet_up_pose(joints) # execute feet up pose
    time.sleep(0.5)

    feet_retrieve_pose(joints)
    time.sleep(1)
    