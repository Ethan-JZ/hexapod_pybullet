# this is some test poses on the robot state
# as for joint 2 on each leg, the angle decreases, the link move upwards
# as for joint 3 on each leg, the angle increases, the link move upwards


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

    