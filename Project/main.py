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

from camera import (
    create_world,
    get_camera_image,
    move_robot,
    close_simulation
)

from perception.obstacle_detection import detect_obstacles
from perception.free_space import detect_free_space

from nav_core.Localization.visualization import Localization
from nav_core.Localization.visual_odometry import VisualOdometry
from nav_core.Localization.trajectory import TrajectoryTracker


# ============================================================
# KEYBOARD CONTROL
# ============================================================

def handle_keyboard(ugv):

    keys = p.getKeyboardEvents()

    speed = 0
    turn = 0

    # Forward
    if ord("w") in keys and keys[ord("w")] & p.KEY_IS_DOWN:
        speed = 5

    # Backward
    if ord("s") in keys and keys[ord("s")] & p.KEY_IS_DOWN:
        speed = -5

    # Left
    if ord("a") in keys and keys[ord("a")] & p.KEY_IS_DOWN:
        turn = 2

    # Right
    if ord("d") in keys and keys[ord("d")] & p.KEY_IS_DOWN:
        turn = -2

    move_robot(ugv, speed, turn)


# ============================================================
# CREATE WORLD
# ============================================================

ugv = create_world()


# ============================================================
# INITIALIZE MODULES
# ============================================================

localization = Localization(ugv)

vo = VisualOdometry()

trajectory = TrajectoryTracker()


# ============================================================
# START MESSAGE
# ============================================================

print()
print("==========================================")
print("        UGV NAVIGATION SYSTEM")
print("==========================================")
print("Camera              : ON")
print("RGB Camera          : ON")
print("Depth Camera        : ON")
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


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        # ----------------------------------------------------
        # CAMERA
        # ----------------------------------------------------

        # Camera now returns TWO things:
        #
        # frame = RGB image
        # depth = depth map
        #
        frame, depth = get_camera_image(ugv)


        # ----------------------------------------------------
        # VISUAL ODOMETRY
        # ----------------------------------------------------

        # For now VO still uses only RGB.
        #
        # Later we will modify VO to use depth.

        vo_x, vo_y, vo_heading = vo.update(frame)


        # ----------------------------------------------------
        # GROUND TRUTH LOCALIZATION
        # ----------------------------------------------------

        gt_x, gt_y, gt_heading = localization.update()


        # ----------------------------------------------------
        # TRAJECTORY TRACKING
        # ----------------------------------------------------

        trajectory.update(
            vo_x,
            vo_y,
            gt_x,
            gt_y
        )


        # ----------------------------------------------------
        # OBSTACLE DETECTION
        # ----------------------------------------------------

        obstacle_frame, obstacle_mask = detect_obstacles(frame)


        # ----------------------------------------------------
        # FREE SPACE DETECTION
        # ----------------------------------------------------

        free_space_frame, free_space_mask = detect_free_space(frame)


        # ====================================================
        # DISPLAY LOCALIZATION INFORMATION
        # ====================================================

        # Visual Odometry

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


        # Ground Truth

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


        # ====================================================
        # DEPTH VISUALIZATION
        # ====================================================

        # Convert depth values into a displayable 0-255 image.

        depth_display = cv2.normalize(
            depth,
            None,
            0,
            255,
            cv2.NORM_MINMAX
        )

        depth_display = depth_display.astype("uint8")


        # ====================================================
        # SHOW WINDOWS
        # ====================================================

        cv2.imshow(
            "UGV Camera - Localization + VO",
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

        cv2.imshow(
            "Depth Camera",
            depth_display
        )


        # ====================================================
        # KEYBOARD
        # ====================================================

        key = cv2.waitKey(1) & 0xFF


        # Quit
        if key == ord("q"):
            break


        # Robot movement
        handle_keyboard(ugv)


        # Physics
        p.stepSimulation()


# ============================================================
# CLEANUP
# ============================================================

finally:

    cv2.destroyAllWindows()

    close_simulation()

    # Show trajectory after quitting

    trajectory.show()