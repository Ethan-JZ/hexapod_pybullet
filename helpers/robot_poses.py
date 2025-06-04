# this is some test poses on the robot state
# as for joint 2 on each leg, the angle decreases, the link move upwards
# as for joint 3 on each leg, the angle increases, the link move upwards

def set_pose(joints, target_joint_position):
    """
    Set the pose of the robot by moving each joint to the target position.
    Input:
    joints: dict, a dictionary containing the joint objects
    target_joint_position: list, a list of target angles for each joint
    Output:
    Moves each joint to the target angle in the specified time.
    """

    joints["11"].move(angle=target_joint_position[0], time=100)
    joints["12"].move(angle=target_joint_position[1], time=100)
    joints["13"].move(angle=target_joint_position[2], time=100)
    joints["41"].move(angle=target_joint_position[3], time=100)
    joints["42"].move(angle=target_joint_position[4], time=100)
    joints["43"].move(angle=target_joint_position[5], time=100)
    joints["21"].move(angle=target_joint_position[6], time=100)
    joints["22"].move(angle=target_joint_position[7], time=100)
    joints["23"].move(angle=target_joint_position[8], time=100)
    joints["51"].move(angle=target_joint_position[9], time=100)
    joints["52"].move(angle=target_joint_position[10], time=100)
    joints["53"].move(angle=target_joint_position[11], time=100)
    joints["61"].move(angle=target_joint_position[12], time=100)
    joints["62"].move(angle=target_joint_position[13], time=100)
    joints["63"].move(angle=target_joint_position[14], time=100)
    joints["31"].move(angle=target_joint_position[15], time=100)
    joints["32"].move(angle=target_joint_position[16], time=100)
    joints["33"].move(angle=target_joint_position[17], time=100)

def feet_up_pose(joints):

    # 1st joint on each leg
    for i in range(1, 7):
        joints[f"{i}1"].move(angle=120, time=100)
    
    # 2nd joint on each leg
    for i in range(1, 7):
        joints[f"{i}2"].move(angle=30, time=100)

    # 3rd joint on each leg
    for i in range(1, 7):
        joints[f"{i}3"].move(angle=60, time=100)

def standing_pose(joints):

    # 1st joint on each leg
    for i in range(1, 7):
        joints[f"{i}1"].move(angle=120, time=100)
    
    # 2nd joint on each leg
    for i in range(1, 7):
        joints[f"{i}2"].move(angle=140, time=100)

    # 3rd joint on each leg
    for i in range(1, 7):
        joints[f"{i}3"].move(angle=60, time=100)


def laying_down_pose(joints):

    # 1st joint on each leg
    for i in range(1, 7):
        joints[f"{i}1"].move(angle=120, time=100)
    
    # 2nd joint on each leg
    for i in range(1, 7):
        joints[f"{i}2"].move(angle=120, time=100)

    # 3rd joint on each leg
    for i in range(1, 7):
        joints[f"{i}3"].move(angle=120, time=100)

    