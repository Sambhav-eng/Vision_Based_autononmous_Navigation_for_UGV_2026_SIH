#-----------------------------------Brain ---------

# main.py
#    │
#    ├── create simulation
#    │
#    ├── get camera image
#    │
#    ├── detect obstacles
#    │
#    ├── detect free space
#    │
#    ├── SLAM
#    │
#    ├── planning
#    │
#    └── control robot

# from camera import create_world, get_camera_image
# from perception.obstacle_detection import detect_obstacles
# from perception.free_space import detect_free_space

# ugv = create_world()

# while True:

#     frame = get_camera_image(ugv)

#     obstacles = detect_obstacles(frame)

#     free_space = detect_free_space(frame)



import cv2
import pybullet as p
import numpy as np

from camera import create_world, get_camera_image, close_simulation
from perception.obstacle_detection import detect_obstacles
from perception.free_space import detect_free_space
from nav_core.Localization.visual_odometry import VisualOdometry


# ============================================================
# START
# ============================================================

ugv = create_world()

# Create Visual Odometry object
vo = VisualOdometry()

print()
print("========================================")
print("       UGV AUTONOMOUS NAVIGATION")
print("========================================")
print("Camera              : ON")
print("Obstacle Detection  : ON")
print("Free Space          : ON")
print("Visual Odometry     : ON")
print("Press Q to quit")
print()


# Store trajectory
trajectory = []

try:

    while True:

        # ====================================================
        # 1. CAMERA
        # ====================================================

        frame = get_camera_image(ugv)

        # ====================================================
        # 2. VISUAL ODOMETRY
        # ====================================================

        x, y, heading = vo.update(frame)

        # Store estimated position
        trajectory.append((x, y))

        # ====================================================
        # 3. OBSTACLE DETECTION
        # ====================================================

        obstacle_frame, obstacle_mask = detect_obstacles(frame)

        # ====================================================
        # 4. FREE SPACE
        # ====================================================

        free_space_frame, free_space_mask = detect_free_space(frame)

        # ====================================================
        # 5. DISPLAY POSITION
        # ====================================================

        cv2.putText(
            obstacle_frame,
            f"X: {x:.2f}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            obstacle_frame,
            f"Y: {y:.2f}",
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            obstacle_frame,
            f"Heading: {heading:.2f}",
            (20, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        # ====================================================
        # 6. DISPLAY
        # ====================================================

        cv2.imshow(
            "UGV Camera - Perception + VO",
            obstacle_frame
        )

        cv2.imshow(
            "Free Space",
            free_space_frame
        )

        cv2.imshow(
            "Obstacle Mask",
            obstacle_mask
        )

        # ====================================================
        # 7. KEYBOARD
        # ====================================================

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        # ====================================================
        # UPDATE PYBULLET
        # ====================================================

        p.stepSimulation()


finally:

    cv2.destroyAllWindows()

    close_simulation()

    print("System stopped.")