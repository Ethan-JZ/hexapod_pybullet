import sympy as sp
import pybullet as p
import time


def remove_small_terms(expr, threshold=1e-13):
    """
    remove small terms in the expression
    Input:
    expr: sympy expression
    threshold: float, the threshold below which terms are considered small and removed
    Output:
    expr: sympy expression with small terms removed
    """
    return expr.xreplace({n: 0 for n in expr.atoms(sp.Number) if abs(n) < threshold})


def get_joint_info(robot_id):
    """
    Get joint information for a robot in PyBullet.
    Input:
    robot_id: urdf object id in pybullet
    Output:
    Prints the total number of joints and their names."""

    num_joints = p.getNumJoints(robot_id)
    print(f"Total joints: {num_joints}")

    # Loop through all joints and print info
    for joint_idx in range(num_joints):
        joint_info = p.getJointInfo(robot_id, joint_idx)
        print(joint_info[0], joint_info[1].decode("utf-8"))

def switch_joint_name_to_index(robot_id, joint_name: str) -> int:
    """
    Convert joint name to index.
    
    Input:
    robot_id: urdf object id in pybullet
    joint_name: str, the name of the joint (e.g., "joint11", "joint12", etc.)
    
    Output:
    joint_index: int, the index of the joint in the robot model.
    """
    # get the number of joints in the robot
    num_joints = p.getNumJoints(robot_id)
    
    for joint_idx in range(num_joints):
        joint_info = p.getJointInfo(robot_id, joint_idx)
        joint_str = joint_info[1].decode("utf-8")

        if joint_str == joint_name:
            return joint_idx
    
    raise ValueError(f"Joint name '{joint_name}' not found in the robot model.")
