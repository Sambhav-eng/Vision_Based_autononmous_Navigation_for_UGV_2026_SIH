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

from camera import create_world, get_camera_image, close_simulation
from perception.obstacle_detection import detect_obstacles
from perception.free_space import detect_free_space


# ============================================================
# START SIMULATION
# ============================================================

ugv = create_world()

print()
print("========================================")
print("     UGV PERCEPTION PIPELINE")
print("========================================")
print("Camera              : ON")
print("Obstacle Detection  : ON")
print("Free Space          : ON")
print("Press Q to quit")
print()


try:

    while True:

        # ----------------------------------------------------
        # 1. GET CAMERA IMAGE
        # ----------------------------------------------------

        frame = get_camera_image(ugv)

        # ----------------------------------------------------
        # 2. OBSTACLE DETECTION
        # ----------------------------------------------------

        obstacle_frame, obstacle_mask = detect_obstacles(frame)

        # ----------------------------------------------------
        # 3. FREE-SPACE DETECTION
        # ----------------------------------------------------

        free_space_frame, free_space_mask = detect_free_space(frame)

        # ----------------------------------------------------
        # 4. DISPLAY RESULTS
        # ----------------------------------------------------

        cv2.imshow(
            "UGV Camera - Obstacles",
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

        # ----------------------------------------------------
        # 5. KEYBOARD
        # ----------------------------------------------------

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        # Update PyBullet
        p.stepSimulation()


finally:


    cv2.destroyAllWindows()

    close_simulation()

    print("System stopped.")