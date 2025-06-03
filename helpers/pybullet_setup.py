import pybullet as p
import pybullet_data


def pybullet_setup(urdf_path):
    """
    This function sets up the PyBullet environment by importing the necessary modules.
    It is designed to be called at the beginning of a script to ensure that all required
    components are available for use.
    """

    # initialization
    physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0, 0, -9.81)  # set gravity
    plane_id = p.loadURDF("plane.urdf") # set plane model
    p.changeVisualShape(plane_id, -1, rgbaColor=[0.5, 0.8, 0.8, 1])
    p.configureDebugVisualizer(flag=p.COV_ENABLE_RENDERING, enable=1, rgbBackground=[1, 1, 1])  # Make sure rendering is on

    # set camera parameters
    cameraDistance = 2.0  # Distance from camera to target
    cameraYaw = 209        # Yaw angle in degrees
    cameraPitch = -27    # Pitch angle in degrees
    cameraTargetPosition = [0, 0, 0]  # Focus point of the camera

    # Reset the debug visualizer camera
    p.resetDebugVisualizerCamera(
        cameraDistance=cameraDistance,
        cameraYaw=cameraYaw,
        cameraPitch=cameraPitch,
        cameraTargetPosition=cameraTargetPosition
    )

    start_pos = [0, 0, 1]
    start_orn = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF(urdf_path, start_pos, start_orn)

    return robot_id