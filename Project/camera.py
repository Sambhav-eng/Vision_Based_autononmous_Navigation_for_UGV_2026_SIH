#What is this file ?
# This version creates a virtual camera that follows the UGV and displays the camera feed in an OpenCV window.




import pybullet as p
import pybullet_data
import cv2
import numpy as np
import time


# ============================================================
# 1. CONNECT TO PYBULLET
# ============================================================

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")


# ============================================================
# 2. CREATE SIMPLE UGV BODY
# ============================================================

body_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[1.0, 0.6, 0.2]
)

body_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[1.0, 0.6, 0.2]
)

ugv = p.createMultiBody(
    baseMass=20,
    baseCollisionShapeIndex=body_collision,
    baseVisualShapeIndex=body_visual,
    basePosition=[0, 0, 0.5]
)


# ============================================================
# 3. CREATE SOME TEST OBJECTS
# ============================================================

def create_box(position, size):

    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=size
    )

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=size
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position
    )


# Objects for the camera to see

create_box([5, 0, 1], [1, 1, 1])

create_box([8, 2, 0.75], [0.75, 0.75, 0.75])

create_box([8, -2, 0.75], [0.75, 0.75, 0.75])


# ============================================================
# 4. CAMERA SETTINGS
# ============================================================

camera_width = 640
camera_height = 480

FOV = 70
NEAR = 0.1
FAR = 50


# ============================================================
# 5. GET CAMERA IMAGE
# ============================================================

def get_camera_image():

    # Get UGV position and orientation
    position, orientation = p.getBasePositionAndOrientation(ugv)

    # Convert orientation to Euler angles
    roll, pitch, yaw = p.getEulerFromQuaternion(
        orientation
    )


    # --------------------------------------------------------
    # Camera position
    # --------------------------------------------------------

    camera_x = position[0] + 0.8 * np.cos(yaw)

    camera_y = position[1] + 0.8 * np.sin(yaw)

    camera_z = position[2] + 0.4


    camera_position = [
        camera_x,
        camera_y,
        camera_z
    ]


    # --------------------------------------------------------
    # Camera looks forward
    # --------------------------------------------------------

    target_x = position[0] + 5 * np.cos(yaw)

    target_y = position[1] + 5 * np.sin(yaw)

    target_z = position[2] + 0.2


    target_position = [
        target_x,
        target_y,
        target_z
    ]


    # --------------------------------------------------------
    # View matrix
    # --------------------------------------------------------

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=target_position,
        cameraUpVector=[0, 0, 1]
    )


    # --------------------------------------------------------
    # Projection matrix
    # --------------------------------------------------------

    projection_matrix = p.computeProjectionMatrixFOV(
        fov=FOV,
        aspect=camera_width / camera_height,
        nearVal=NEAR,
        farVal=FAR
    )


    # --------------------------------------------------------
    # Render camera image
    # --------------------------------------------------------

    image = p.getCameraImage(
        width=camera_width,
        height=camera_height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )


    # Extract RGB image
    rgb = image[2]

    # Convert to NumPy array
    rgb = np.array(rgb, dtype=np.uint8)

    # Reshape
    rgb = rgb.reshape(
        camera_height,
        camera_width,
        4
    )

    # Remove alpha channel
    rgb = rgb[:, :, :3]

    # PyBullet uses RGB
    # OpenCV expects BGR
    bgr = cv2.cvtColor(
        rgb,
        cv2.COLOR_RGB2BGR
    )

    return bgr


# ============================================================
# 6. KEYBOARD CONTROL
# ============================================================

speed = 2.0
turn_speed = 1.5


print()
print("======================================")
print("          UGV CAMERA TEST")
print("======================================")
print()
print("W -> Forward")
print("S -> Backward")
print("A -> Turn Left")
print("D -> Turn Right")
print("Q -> Quit")
print()
print("======================================")


# ============================================================
# 7. MAIN LOOP
# ============================================================

while p.isConnected():

    keys = p.getKeyboardEvents()


    # Default velocity
    linear_velocity = [0, 0, 0]
    angular_velocity = [0, 0, 0]


    # Forward
    if ord("w") in keys and keys[ord("w")] & p.KEY_IS_DOWN:

        linear_velocity = [speed, 0, 0]


    # Backward
    elif ord("s") in keys and keys[ord("s")] & p.KEY_IS_DOWN:

        linear_velocity = [-speed, 0, 0]


    # Left
    elif ord("a") in keys and keys[ord("a")] & p.KEY_IS_DOWN:

        angular_velocity = [0, 0, turn_speed]


    # Right
    elif ord("d") in keys and keys[ord("d")] & p.KEY_IS_DOWN:

        angular_velocity = [0, 0, -turn_speed]


    # Apply velocity
    p.resetBaseVelocity(
        ugv,
        linearVelocity=linear_velocity,
        angularVelocity=angular_velocity
    )


    # Step physics
    p.stepSimulation()


    # ========================================================
    # CAMERA
    # ========================================================

    frame = get_camera_image()


    # Display camera feed
    cv2.imshow(
        "UGV Camera",
        frame
    )


    # ========================================================
    # QUIT
    # ========================================================

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):

        break


    time.sleep(1 / 240)


# ============================================================
# 8. CLEANUP
# ============================================================

cv2.destroyAllWindows()

p.disconnect();
