#What is this file ?
#-------------------------------------Architecture of this file --------------------------------------------------------------
# camera.py                                                                                                                   
# │
# ├── create_background()
# │
# ├── create_world()
# │      │
# │      ├── Ground
# │      ├── Background
# │      ├── UGV
# │      └── Obstacles
# │
# ├── create_obstacle()
# │
# ├── get_camera_image()
# │
# ├── move_robot()
# │
# └── close_simulation()
#-------------------------------------------------------------------------------------------------------------------------------
# This version creates a virtual camera that follows the UGV and displays the camera feed in an OpenCV window.
import pybullet as p
import pybullet_data
import cv2
import numpy as np
import os


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

FOV = 70
NEAR_PLANE = 0.1
FAR_PLANE = 50



#-----------------------------------------------Adding terrain to the simulation-------------------------------------------------
# Terrain/ground/robot visual helpers now live in terrain_utils.py so camera.py
# and simulation.py can't drift out of sync with each other again.
from terrain_utils import (
    get_horizon_texture_id,
    get_ground_texture_id,
    apply_ground_terrain,
    create_backdrop_wall,
    create_wheeled_ugv,
    apply_realistic_scene_settings,
)



def create_world():

    # Start PyBullet
    if not p.isConnected():
        p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.resetSimulation()

    p.setGravity(0, 0, -9.81)

    # Ground
    plane_id = p.loadURDF("plane.urdf")

    project_dir = os.path.dirname(__file__)

    # Seamless generated grass texture for the ground the UGV drives on
    ground_texture_id = get_ground_texture_id(project_dir)
    apply_ground_terrain(plane_id, ground_texture_id)

    # Your landscape photo goes on the distant backdrop wall instead -
    # that's what it's actually suited for (sky/mountains/horizon)
    horizon_texture_id = get_horizon_texture_id(project_dir)
    create_backdrop_wall(horizon_texture_id)

    # Shadows + a nicely framed camera so the scene reads as 3D
    apply_realistic_scene_settings()

    # -------------------------------------------------
    # UGV - proper wheeled chassis instead of a flat box
    # -------------------------------------------------

    ugv = create_wheeled_ugv(base_position=(0, 0, 0.35))

    # -------------------------------------------------
    # TEST OBSTACLES
    # -------------------------------------------------

    create_obstacle([5, 0, 1], [0.5, 0.5, 1])
    create_obstacle([8, 2, 0.75], [0.7, 0.7, 0.75])
    create_obstacle([8, -2, 0.75], [0.7, 0.7, 0.75])

    return ugv


def create_obstacle(position, half_extents):

    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents
    )

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=[1, 0, 0, 1]  # kept pure red - obstacle_detection.py thresholds on this color
    )

    obstacle = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position
    )

    return obstacle


def get_camera_image(ugv):

    # Get robot position
    position, orientation = p.getBasePositionAndOrientation(ugv)

    # Get robot yaw
    _, _, yaw = p.getEulerFromQuaternion(orientation)

    x, y, z = position

    # -------------------------------------------------
    # CAMERA POSITION
    # -------------------------------------------------

    camera_distance = 0.8

    camera_x = x + camera_distance * np.cos(yaw)
    camera_y = y + camera_distance * np.sin(yaw)
    camera_z = z + 0.4

    camera_position = [
        camera_x,
        camera_y,
        camera_z
    ]

    # -------------------------------------------------
    # CAMERA TARGET
    # -------------------------------------------------

    target_distance = 5

    target_x = x + target_distance * np.cos(yaw)
    target_y = y + target_distance * np.sin(yaw)
    target_z = z

    target_position = [
        target_x,
        target_y,
        target_z
    ]

    # -------------------------------------------------
    # VIEW MATRIX
    # -------------------------------------------------

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=target_position,
        cameraUpVector=[0, 0, 1]
    )

    # -------------------------------------------------
    # PROJECTION MATRIX
    # -------------------------------------------------

    projection_matrix = p.computeProjectionMatrixFOV(
        fov=FOV,
        aspect=CAMERA_WIDTH / CAMERA_HEIGHT,
        nearVal=NEAR_PLANE,
        farVal=FAR_PLANE
    )

    # -------------------------------------------------
    # CAPTURE IMAGE
    # -------------------------------------------------

    image = p.getCameraImage(
        CAMERA_WIDTH,
        CAMERA_HEIGHT,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )

    rgba = image[2]

    frame = np.array(
        rgba,
        dtype=np.uint8
    )

    # Important reshape
    frame = frame.reshape(
        CAMERA_HEIGHT,
        CAMERA_WIDTH,
        4
    )

    # Remove alpha channel
    frame = frame[:, :, :3]

    # RGB -> BGR for OpenCV
    frame = cv2.cvtColor(
        frame,
        cv2.COLOR_RGB2BGR
    )

    return frame


def move_robot(ugv, speed, turn=0):

    position, orientation = p.getBasePositionAndOrientation(ugv)

    _, _, yaw = p.getEulerFromQuaternion(orientation)

    linear_velocity = [
        speed * np.cos(yaw),
        speed * np.sin(yaw),
        0
    ]

    angular_velocity = [
        0,
        0,
        turn
    ]

    p.resetBaseVelocity(
        ugv,
        linearVelocity=linear_velocity,
        angularVelocity=angular_velocity
    )


def close_simulation():

    if p.isConnected():
        p.disconnect()






#-----------------------------------------------OLD ----------------------------------------------------------------------
# import pybullet as p
# import pybullet_data
# import cv2
# import numpy as np
# import time


# # ============================================================
# # 1. CONNECT TO PYBULLET
# # ============================================================

# p.connect(p.GUI)

# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.setGravity(0, 0, -9.81)

# p.loadURDF("plane.urdf")


# # ============================================================
# # 2. CREATE SIMPLE UGV BODY
# # ============================================================

# body_collision = p.createCollisionShape(
#     p.GEOM_BOX,
#     halfExtents=[1.0, 0.6, 0.2]
# )

# body_visual = p.createVisualShape(
#     p.GEOM_BOX,
#     halfExtents=[1.0, 0.6, 0.2]
# )

# ugv = p.createMultiBody(
#     baseMass=20,
#     baseCollisionShapeIndex=body_collision,
#     baseVisualShapeIndex=body_visual,
#     basePosition=[0, 0, 0.5]
# )


# # ============================================================
# # 3. CREATE SOME TEST OBJECTS
# # ============================================================

# def create_box(position, size):

#     collision = p.createCollisionShape(
#         p.GEOM_BOX,
#         halfExtents=size
#     )

#     visual = p.createVisualShape(
#         p.GEOM_BOX,
#         halfExtents=size
#     )

#     return p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=collision,
#         baseVisualShapeIndex=visual,
#         basePosition=position
#     )


# # Objects for the camera to see

# create_box([5, 0, 1], [1, 1, 1])

# create_box([8, 2, 0.75], [0.75, 0.75, 0.75])

# create_box([8, -2, 0.75], [0.75, 0.75, 0.75])


# # ============================================================
# # 4. CAMERA SETTINGS
# # ============================================================

# camera_width = 640
# camera_height = 480

# FOV = 70
# NEAR = 0.1
# FAR = 50


# # ============================================================
# # 5. GET CAMERA IMAGE
# # ============================================================

# def get_camera_image():

#     # Get UGV position and orientation
#     position, orientation = p.getBasePositionAndOrientation(ugv)

#     # Convert orientation to Euler angles
#     roll, pitch, yaw = p.getEulerFromQuaternion(
#         orientation
#     )


#     # --------------------------------------------------------
#     # Camera position
#     # --------------------------------------------------------

#     camera_x = position[0] + 0.8 * np.cos(yaw)

#     camera_y = position[1] + 0.8 * np.sin(yaw)

#     camera_z = position[2] + 0.4


#     camera_position = [
#         camera_x,
#         camera_y,
#         camera_z
#     ]


#     # --------------------------------------------------------
#     # Camera looks forward
#     # --------------------------------------------------------

#     target_x = position[0] + 5 * np.cos(yaw)

#     target_y = position[1] + 5 * np.sin(yaw)

#     target_z = position[2] + 0.2


#     target_position = [
#         target_x,
#         target_y,
#         target_z
#     ]


#     # --------------------------------------------------------
#     # View matrix
#     # --------------------------------------------------------

#     view_matrix = p.computeViewMatrix(
#         cameraEyePosition=camera_position,
#         cameraTargetPosition=target_position,
#         cameraUpVector=[0, 0, 1]
#     )


#     # --------------------------------------------------------
#     # Projection matrix
#     # --------------------------------------------------------

#     projection_matrix = p.computeProjectionMatrixFOV(
#         fov=FOV,
#         aspect=camera_width / camera_height,
#         nearVal=NEAR,
#         farVal=FAR
#     )


#     # --------------------------------------------------------
#     # Render camera image
#     # --------------------------------------------------------

#     image = p.getCameraImage(
#         width=camera_width,
#         height=camera_height,
#         viewMatrix=view_matrix,
#         projectionMatrix=projection_matrix,
#         renderer=p.ER_BULLET_HARDWARE_OPENGL
#     )


#     # Extract RGB image
#     rgb = image[2]

#     # Convert to NumPy array
#     rgb = np.array(rgb, dtype=np.uint8)

#     # Reshape
#     rgb = rgb.reshape(
#         camera_height,
#         camera_width,
#         4
#     )

#     # Remove alpha channel
#     rgb = rgb[:, :, :3]

#     # PyBullet uses RGB
#     # OpenCV expects BGR
#     bgr = cv2.cvtColor(
#         rgb,
#         cv2.COLOR_RGB2BGR
#     )

#     return bgr


# # ============================================================
# # 6. KEYBOARD CONTROL
# # ============================================================

# speed = 2.0
# turn_speed = 1.5


# print()
# print("======================================")
# print("          UGV CAMERA TEST")
# print("======================================")
# print()
# print("W -> Forward")
# print("S -> Backward")
# print("A -> Turn Left")
# print("D -> Turn Right")
# print("Q -> Quit")
# print()
# print("======================================")


# # ============================================================
# # 7. MAIN LOOP
# # ============================================================

# while p.isConnected():

#     keys = p.getKeyboardEvents()


#     # Default velocity
#     linear_velocity = [0, 0, 0]
#     angular_velocity = [0, 0, 0]


#     # Forward
#     if ord("w") in keys and keys[ord("w")] & p.KEY_IS_DOWN:

#         linear_velocity = [speed, 0, 0]


#     # Backward
#     elif ord("s") in keys and keys[ord("s")] & p.KEY_IS_DOWN:

#         linear_velocity = [-speed, 0, 0]


#     # Left
#     elif ord("a") in keys and keys[ord("a")] & p.KEY_IS_DOWN:

#         angular_velocity = [0, 0, turn_speed]


#     # Right
#     elif ord("d") in keys and keys[ord("d")] & p.KEY_IS_DOWN:

#         angular_velocity = [0, 0, -turn_speed]


#     # Apply velocity
#     p.resetBaseVelocity(
#         ugv,
#         linearVelocity=linear_velocity,
#         angularVelocity=angular_velocity
#     )


#     # Step physics
#     p.stepSimulation()


#     # ========================================================
#     # CAMERA
#     # ========================================================

#     frame = get_camera_image()


#     # Display camera feed
#     cv2.imshow(
#         "UGV Camera",
#         frame
#     )


#     # ========================================================
#     # QUIT
#     # ========================================================

#     key = cv2.waitKey(1) & 0xFF

#     if key == ord("q"):

#         break


#     time.sleep(1 / 240)


# # ============================================================
# # 8. CLEANUP
# # ============================================================

# cv2.destroyAllWindows()

# p.disconnect();
