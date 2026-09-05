import cv2
import numpy as np


def detect_obstacles(frame):

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    # Red range 1
    lower_red_1 = np.array([0, 100, 80])
    upper_red_1 = np.array([10, 255, 255])

    # Red range 2
    lower_red_2 = np.array([170, 100, 80])
    upper_red_2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(
        hsv,
        lower_red_1,
        upper_red_1
    )

    mask2 = cv2.inRange(
        hsv,
        lower_red_2,
        upper_red_2
    )

    # Combine masks
    mask = mask1 | mask2

    # Remove noise
    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    # Find obstacles
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    obstacle_count = 0

    output = frame.copy()

    for contour in contours:

        area = cv2.contourArea(contour)

        if area < 300:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        obstacle_count += 1

        # Draw bounding box
        cv2.rectangle(
            output,
            (x, y),
            (x + w, y + h),
            (0, 255, 0),
            2
        )

        # Center
        center_x = x + w // 2
        center_y = y + h // 2

        cv2.circle(
            output,
            (center_x, center_y),
            5,
            (255, 0, 0),
            -1
        )

        # Label
        cv2.putText(
            output,
            "OBSTACLE",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    # Count
    cv2.putText(
        output,
        f"Obstacles: {obstacle_count}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    return output, mask










#-------------------------------Old code ----------------------------------------------------#

# import pybullet as p
# import pybullet_data
# import cv2
# import numpy as np
# import time


# # ============================================================
# # 1. START PYBULLET
# # ============================================================

# physics_client = p.connect(p.GUI)

# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.setGravity(0, 0, -9.81)

# # Ground
# p.loadURDF("plane.urdf")


# # ============================================================
# # 2. CREATE UGV
# # ============================================================

# ugv_collision = p.createCollisionShape(
#     p.GEOM_BOX,
#     halfExtents=[1.0, 0.7, 0.3]
# )

# ugv_visual = p.createVisualShape(
#     p.GEOM_BOX,
#     halfExtents=[1.0, 0.7, 0.3],
#     rgbaColor=[0.2, 0.2, 0.2, 1]
# )

# ugv = p.createMultiBody(
#     baseMass=20,
#     baseCollisionShapeIndex=ugv_collision,
#     baseVisualShapeIndex=ugv_visual,
#     basePosition=[0, 0, 0.5]
# )


# # ============================================================
# # 3. CREATE RED OBSTACLE FUNCTION
# # ============================================================

# def create_obstacle(position, size):

#     collision_shape = p.createCollisionShape(
#         p.GEOM_BOX,
#         halfExtents=size
#     )

#     visual_shape = p.createVisualShape(
#         p.GEOM_BOX,
#         halfExtents=size,
#         rgbaColor=[1, 0, 0, 1]
#     )

#     obstacle = p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=collision_shape,
#         baseVisualShapeIndex=visual_shape,
#         basePosition=position
#     )

#     return obstacle


# # ============================================================
# # 4. CREATE OBSTACLES
# # ============================================================

# # Large obstacle directly ahead
# create_obstacle(
#     position=[5, 0, 1],
#     size=[1, 1, 1]
# )

# # Obstacle on the left
# create_obstacle(
#     position=[8, 2, 0.75],
#     size=[0.75, 0.75, 0.75]
# )

# # Obstacle on the right
# create_obstacle(
#     position=[8, -2, 0.75],
#     size=[0.75, 0.75, 0.75]
# )


# # ============================================================
# # 5. CAMERA SETTINGS
# # ============================================================

# WIDTH = 640
# HEIGHT = 480

# FOV = 70

# NEAR = 0.1
# FAR = 50.0


# # ============================================================
# # 6. GET IMAGE FROM VIRTUAL CAMERA
# # ============================================================

# def get_camera_image():

#     # Get UGV position and orientation
#     position, orientation = p.getBasePositionAndOrientation(ugv)

#     # Convert quaternion to rotation matrix
#     rotation = np.array(
#         p.getMatrixFromQuaternion(orientation)
#     ).reshape(3, 3)

#     # --------------------------------------------------------
#     # Camera position
#     # --------------------------------------------------------

#     camera_position = np.array(position)

#     # Put camera above the UGV
#     camera_position[2] += 0.8

#     # UGV forward direction
#     forward = rotation @ np.array([1, 0, 0])

#     # Move camera slightly forward
#     camera_position += forward * 0.5

#     # Camera looks forward
#     target_position = camera_position + forward * 8

#     # Up direction
#     up = rotation @ np.array([0, 0, 1])

#     # --------------------------------------------------------
#     # View matrix
#     # --------------------------------------------------------

#     view_matrix = p.computeViewMatrix(
#         cameraEyePosition=camera_position,
#         cameraTargetPosition=target_position,
#         cameraUpVector=up
#     )

#     # --------------------------------------------------------
#     # Projection matrix
#     # --------------------------------------------------------

#     projection_matrix = p.computeProjectionMatrixFOV(
#         fov=FOV,
#         aspect=WIDTH / HEIGHT,
#         nearVal=NEAR,
#         farVal=FAR
#     )

#     # --------------------------------------------------------
#     # Capture image
#     # --------------------------------------------------------

#     result = p.getCameraImage(
#         width=WIDTH,
#         height=HEIGHT,
#         viewMatrix=view_matrix,
#         projectionMatrix=projection_matrix,
#         renderer=p.ER_BULLET_HARDWARE_OPENGL
#     )

#     rgb_data = result[2]

#     # Convert PyBullet output to NumPy array
#     image = np.array(rgb_data, dtype=np.uint8)

#     # --------------------------------------------------------
#     # IMPORTANT:
#     # PyBullet can return the image as a flat array.
#     # We explicitly reshape it.
#     # --------------------------------------------------------

#     expected_rgba_size = WIDTH * HEIGHT * 4

#     if image.size == expected_rgba_size:

#         image = image.reshape(
#             HEIGHT,
#             WIDTH,
#             4
#         )

#         # Remove Alpha channel
#         image = image[:, :, :3]

#     else:

#         # Fallback if PyBullet already returned 3D data
#         image = image.reshape(
#             HEIGHT,
#             WIDTH,
#             -1
#         )

#         image = image[:, :, :3]

#     # RGB → BGR
#     image = cv2.cvtColor(
#         image,
#         cv2.COLOR_RGB2BGR
#     )

#     return image


# # ============================================================
# # 7. DETECT RED OBSTACLES
# # ============================================================

# def detect_obstacles(frame):

#     # Convert BGR → HSV
#     hsv = cv2.cvtColor(
#         frame,
#         cv2.COLOR_BGR2HSV
#     )

#     # --------------------------------------------------------
#     # RED COLOR RANGE
#     # --------------------------------------------------------

#     lower_red_1 = np.array([0, 100, 80])
#     upper_red_1 = np.array([10, 255, 255])

#     lower_red_2 = np.array([170, 100, 80])
#     upper_red_2 = np.array([180, 255, 255])

#     mask1 = cv2.inRange(
#         hsv,
#         lower_red_1,
#         upper_red_1
#     )

#     mask2 = cv2.inRange(
#         hsv,
#         lower_red_2,
#         upper_red_2
#     )

#     # Combine both red ranges
#     mask = mask1 | mask2

#     # --------------------------------------------------------
#     # REMOVE SMALL NOISE
#     # --------------------------------------------------------

#     kernel = np.ones(
#         (5, 5),
#         np.uint8
#     )

#     mask = cv2.morphologyEx(
#         mask,
#         cv2.MORPH_OPEN,
#         kernel
#     )

#     mask = cv2.morphologyEx(
#         mask,
#         cv2.MORPH_CLOSE,
#         kernel
#     )

#     # --------------------------------------------------------
#     # FIND OBJECT CONTOURS
#     # --------------------------------------------------------

#     contours, _ = cv2.findContours(
#         mask,
#         cv2.RETR_EXTERNAL,
#         cv2.CHAIN_APPROX_SIMPLE
#     )

#     obstacle_count = 0

#     for contour in contours:

#         area = cv2.contourArea(contour)

#         # Ignore tiny regions
#         if area < 300:
#             continue

#         # Bounding box
#         x, y, w, h = cv2.boundingRect(contour)

#         obstacle_count += 1

#         # ----------------------------------------------------
#         # Draw bounding box
#         # ----------------------------------------------------

#         cv2.rectangle(
#             frame,
#             (x, y),
#             (x + w, y + h),
#             (0, 255, 0),
#             2
#         )

#         # ----------------------------------------------------
#         # Calculate center
#         # ----------------------------------------------------

#         center_x = x + w // 2
#         center_y = y + h // 2

#         cv2.circle(
#             frame,
#             (center_x, center_y),
#             5,
#             (255, 0, 0),
#             -1
#         )

#         # ----------------------------------------------------
#         # Label
#         # ----------------------------------------------------

#         cv2.putText(
#             frame,
#             "OBSTACLE",
#             (x, y - 10),
#             cv2.FONT_HERSHEY_SIMPLEX,
#             0.6,
#             (0, 255, 0),
#             2
#         )

#     # --------------------------------------------------------
#     # Display obstacle count
#     # --------------------------------------------------------

#     cv2.putText(
#         frame,
#         f"Obstacles: {obstacle_count}",
#         (20, 35),
#         cv2.FONT_HERSHEY_SIMPLEX,
#         0.8,
#         (255, 255, 255),
#         2
#     )

#     return frame, mask


# # ============================================================
# # 8. MAIN LOOP
# # ============================================================

# print()
# print("========================================")
# print("   UGV OBSTACLE DETECTION")
# print("========================================")
# print("Camera started.")
# print("Red objects = obstacles")
# print("Press Q to quit.")
# print()


# try:

#     while True:

#         # ----------------------------------------------------
#         # Get camera image
#         # ----------------------------------------------------

#         frame = get_camera_image()

#         # ----------------------------------------------------
#         # Detect obstacles
#         # ----------------------------------------------------

#         detected_frame, mask = detect_obstacles(frame)

#         # ----------------------------------------------------
#         # Display camera
#         # ----------------------------------------------------

#         cv2.imshow(
#             "UGV Camera - Obstacle Detection",
#             detected_frame
#         )

#         # ----------------------------------------------------
#         # Display mask
#         # ----------------------------------------------------

#         cv2.imshow(
#             "Obstacle Mask",
#             mask
#         )

#         # ----------------------------------------------------
#         # Quit with Q
#         # ----------------------------------------------------

#         key = cv2.waitKey(1) & 0xFF

#         if key == ord("q"):
#             break

#         # Give PyBullet time to update
#         p.stepSimulation()

#         time.sleep(1 / 60)


# finally:

#     cv2.destroyAllWindows()

#     if p.isConnected():
#         p.disconnect()

#     print("Obstacle detection stopped.")