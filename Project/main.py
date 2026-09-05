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


# ==========================================
# Camera
# ==========================================

from camera import (
    create_world,
    get_camera_image,
    move_robot,
    close_simulation
)


# ==========================================
# Perception
# ==========================================

from perception.obstacle_detection import (
    detect_obstacles
)

from perception.free_space import (
    detect_free_space
)


# ==========================================
# Localization
# ==========================================

from nav_core.Localization.visualization import (
    Localization
)

from nav_core.Localization.visual_odometry import (
    VisualOdometry
)

from nav_core.Localization.trajectory import (
    TrajectoryTracker
)


# ==========================================
# Keyboard Control
# ==========================================

def handle_keyboard(ugv):

    keys = p.getKeyboardEvents()

    speed = 0
    turn = 0

    # --------------------------
    # Forward
    # --------------------------

    if (
        ord("w") in keys
        and keys[ord("w")] & p.KEY_IS_DOWN
    ):
        speed = 5

    # --------------------------
    # Backward
    # --------------------------

    if (
        ord("s") in keys
        and keys[ord("s")] & p.KEY_IS_DOWN
    ):
        speed = -5

    # --------------------------
    # Left
    # --------------------------

    if (
        ord("a") in keys
        and keys[ord("a")] & p.KEY_IS_DOWN
    ):
        turn = 2

    # --------------------------
    # Right
    # --------------------------

    if (
        ord("d") in keys
        and keys[ord("d")] & p.KEY_IS_DOWN
    ):
        turn = -2

    # Send movement command
    move_robot(
        ugv,
        speed,
        turn
    )


# ==========================================
# Create World
# ==========================================

ugv = create_world()


# ==========================================
# Create Localization
# ==========================================

localization = Localization(
    ugv
)


# ==========================================
# Create Visual Odometry
# ==========================================

vo = VisualOdometry()


# ==========================================
# Create Trajectory Tracker
# ==========================================

trajectory = TrajectoryTracker()


# ==========================================
# Startup message
# ==========================================

print()
print("==========================================")
print("        UGV NAVIGATION SYSTEM")
print("==========================================")
print("Camera              : ON")
print("Obstacle Detection  : ON")
print("Free Space          : ON")
print("Localization        : ON")
print("Visual Odometry     : ON")
print("Trajectory Tracking : ON")
print("------------------------------------------")
print("W = Forward")
print("S = Backward")
print("A = Left")
print("D = Right")
print("Q = Quit")
print("==========================================")
print()


# ==========================================
# Main Loop
# ==========================================

try:

    while True:

        # ==================================
        # 1. Camera
        # ==================================

        frame = get_camera_image(
            ugv
        )


        # ==================================
        # 2. Visual Odometry
        # ==================================

        vo_x, vo_y, vo_heading = vo.update(
            frame
        )


        # ==================================
        # 3. Ground Truth Localization
        # ==================================

        gt_x, gt_y, gt_heading = (
            localization.update()
        )


        # ==================================
        # 4. Save trajectory
        # ==================================

        trajectory.update(
            vo_x,
            vo_y,
            gt_x,
            gt_y
        )


        # ==================================
        # 5. Obstacle Detection
        # ==================================

        obstacle_frame, obstacle_mask = (
            detect_obstacles(
                frame
            )
        )


        # ==================================
        # 6. Free Space Detection
        # ==================================

        free_space_frame, free_space_mask = (
            detect_free_space(
                frame
            )
        )


        # ==================================
        # 7. Display VO
        # ==================================

        cv2.putText(
            obstacle_frame,
            f"VO X: {vo_x:.2f}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            obstacle_frame,
            f"VO Y: {vo_y:.2f}",
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            obstacle_frame,
            f"VO Heading: {vo_heading:.2f}",
            (20, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )


        # ==================================
        # 8. Display Ground Truth
        # ==================================

        cv2.putText(
            obstacle_frame,
            f"GT X: {gt_x:.2f}",
            (350, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.putText(
            obstacle_frame,
            f"GT Y: {gt_y:.2f}",
            (350, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.putText(
            obstacle_frame,
            f"GT Heading: {gt_heading:.2f}",
            (350, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )


        # ==================================
        # 9. Display Camera
        # ==================================

        cv2.imshow(
            "UGV Camera - Localization + VO",
            obstacle_frame
        )


        # ==================================
        # 10. Display Free Space
        # ==================================

        cv2.imshow(
            "Free Space",
            free_space_frame
        )


        # ==================================
        # 11. Display Obstacle Mask
        # ==================================

        cv2.imshow(
            "Obstacle Mask",
            obstacle_mask
        )


        # ==================================
        # 12. Keyboard
        # ==================================

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):

            break


        # ==================================
        # 13. Move Robot
        # ==================================

        handle_keyboard(
            ugv
        )


        # ==================================
        # 14. Physics
        # ==================================

        p.stepSimulation()


finally:

    # ======================================
    # Close OpenCV windows
    # ======================================

    cv2.destroyAllWindows()


    # ======================================
    # Close PyBullet
    # ======================================

    close_simulation()


    # ======================================
    # Show trajectory
    # ======================================

    trajectory.show()