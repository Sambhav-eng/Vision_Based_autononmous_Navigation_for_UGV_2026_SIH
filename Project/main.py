# ============================================================
# main.py
#
# Brain of the UGV Navigation System
#
# Camera
#   ↓
# Perception
#   ↓
# Localization
#   ↓
# Mapping
#   ↓
# Planning
#   ↓
# Control
# ============================================================

import numpy as np
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

from nav_core.Mapping.occupancy_grid import OccupancyGrid


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

    move_robot(
        ugv,
        speed,
        turn
    )


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

mapping = OccupancyGrid()


# ============================================================
# TEMPORAL OBSTACLE TRACKING
#
# These tracks are temporary.
#
# The occupancy grid remains persistent.
# ============================================================

obstacle_tracks = []


# Maximum distance between two detections for them
# to be considered the same physical obstacle.
TRACK_DISTANCE_THRESHOLD = 0.7


# Exponential smoothing factor.
#
# Smaller value:
#   More stable
#   Slower response
#
# Larger value:
#   Faster response
#   More sensitive to noise
#
SMOOTHING_ALPHA = 0.2


# Minimum number of observations before an
# obstacle is considered stable.
MIN_OBSERVATIONS = 5


# ============================================================
# UPDATE OBSTACLE TRACKS
# ============================================================

def update_obstacle_tracks(
    detections
):
    """
    detections:
        List of world coordinates:
        [(x1, y1), (x2, y2), ...]

    The function does NOT clear the occupancy grid.

    It maintains temporary obstacle tracks so that
    noisy frame-to-frame measurements do not create
    smeared obstacle trails.
    """

    for x, y in detections:

        best_track = None
        best_distance = float("inf")

        # ----------------------------------------------------
        # Find closest existing UNCONFIRMED track
        # ----------------------------------------------------

        for track in obstacle_tracks:

            # IMPORTANT:
            #
            # Confirmed tracks are already stored in the
            # persistent map. We do not move them anymore.
            #
            if track["confirmed"]:
                continue

            distance = np.sqrt(
                (x - track["x"]) ** 2
                +
                (y - track["y"]) ** 2
            )

            if (
                distance < TRACK_DISTANCE_THRESHOLD
                and distance < best_distance
            ):

                best_track = track
                best_distance = distance


        # ----------------------------------------------------
        # Existing track
        # ----------------------------------------------------

        if best_track is not None:

            # Smooth X position
            best_track["x"] = (
                (1 - SMOOTHING_ALPHA)
                * best_track["x"]
                +
                SMOOTHING_ALPHA
                * x
            )

            # Smooth Y position
            best_track["y"] = (
                (1 - SMOOTHING_ALPHA)
                * best_track["y"]
                +
                SMOOTHING_ALPHA
                * y
            )

            best_track["observations"] += 1


        # ----------------------------------------------------
        # New track
        # ----------------------------------------------------

        else:

            obstacle_tracks.append(
                {
                    "x": x,
                    "y": y,
                    "observations": 1,

                    # False means this obstacle has NOT
                    # yet been permanently inserted.
                    "confirmed": False
                }
            )


# ============================================================
# CONFIRM STABLE OBSTACLES
# ============================================================

def commit_stable_obstacles():

    for track in obstacle_tracks:

        # ----------------------------------------------------
        # IMPORTANT:
        #
        # If already confirmed, do nothing.
        #
        # This prevents the same moving/smoothed track from
        # being painted repeatedly into the persistent map.
        # ----------------------------------------------------

        if track["confirmed"]:
            continue


        # ----------------------------------------------------
        # Wait for enough observations
        # ----------------------------------------------------

        if (
            track["observations"]
            >= MIN_OBSERVATIONS
        ):

            # ----------------------------------------------
            # Mark track as confirmed
            # ----------------------------------------------

            track["confirmed"] = True


            # ----------------------------------------------
            # Write to persistent map ONLY ONCE
            # ----------------------------------------------

            mapping.mark_obstacle_area(
                track["x"],
                track["y"],
                radius=0.3
            )


            print(
                f"[MAPPING] Obstacle confirmed: "
                f"X={track['x']:.2f}, "
                f"Y={track['y']:.2f}"
            )


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
print("Mapping             : ON")
print("Persistent Mapping  : ON")
print("Temporal Filtering  : ON")
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

        # ====================================================
        # CAMERA
        # ====================================================

        frame, depth = get_camera_image(
            ugv
        )


        # ====================================================
        # VISUAL ODOMETRY
        # ====================================================

        vo_x, vo_y, vo_heading = vo.update(
            frame
        )


        # ====================================================
        # LOCALIZATION
        # ====================================================

        gt_x, gt_y, gt_heading = localization.update()


        # ====================================================
        # TRAJECTORY TRACKING
        # ====================================================

        trajectory.update(
            vo_x,
            vo_y,
            gt_x,
            gt_y
        )


        # ====================================================
        # OBSTACLE DETECTION
        # ====================================================

        obstacle_frame, obstacle_mask, obstacles = detect_obstacles(
            frame
        )


        # ====================================================
        # FREE SPACE DETECTION
        # ====================================================

        free_space_frame, free_space_mask = detect_free_space(
            frame
        )


        # ====================================================
        # MAPPING
        #
        # IMPORTANT:
        #
        # We first calculate world-coordinate observations.
        #
        # We do NOT immediately write them to the
        # persistent occupancy grid.
        # ====================================================

        world_obstacles = []


        for obstacle in obstacles:

            # ------------------------------------------------
            # Get bottom-center pixel
            # ------------------------------------------------

            pixel_x, pixel_y = obstacle[
                "bottom_center"
            ]


            # ------------------------------------------------
            # Check pixel is inside depth image
            # ------------------------------------------------

            if (
                pixel_x < 0
                or pixel_x >= depth.shape[1]
                or pixel_y < 0
                or pixel_y >= depth.shape[0]
            ):
                continue


            # ------------------------------------------------
            # Get depth
            # ------------------------------------------------

            obstacle_depth = float(
                depth[
                    pixel_y,
                    pixel_x
                ]
            )


            # ------------------------------------------------
            # Ignore invalid depth
            # ------------------------------------------------

            if not np.isfinite(
                obstacle_depth
            ):
                continue

            if obstacle_depth <= 0:
                continue

            if obstacle_depth > 20:
                continue


            # ------------------------------------------------
            # Convert pixel + depth into world coordinates
            # ------------------------------------------------

            world_x, world_y = mapping.obstacle_to_world(

                pixel_x,
                pixel_y,

                obstacle_depth,

                gt_x,
                gt_y,
                gt_heading
            )


            # ------------------------------------------------
            # Store observation
            #
            # This is temporary.
            # ------------------------------------------------

            world_obstacles.append(
                (
                    world_x,
                    world_y
                )
            )


        # ====================================================
        # TEMPORAL FILTER
        # ====================================================

        update_obstacle_tracks(
            world_obstacles
        )


        # ====================================================
        # CONFIRM STABLE OBSTACLES
        # ====================================================

        commit_stable_obstacles()


        # ====================================================
        # CREATE MAP IMAGE
        # ====================================================

        map_image = mapping.get_map_image(
            robot_x=gt_x,
            robot_y=gt_y,
            robot_heading=gt_heading
        )


        # ====================================================
        # LOCALIZATION INFORMATION
        # ====================================================

        # ----------------------------------------------------
        # Visual Odometry
        # ----------------------------------------------------

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


        # ----------------------------------------------------
        # Ground Truth
        # ----------------------------------------------------

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
        # MAPPING INFORMATION
        # ====================================================

        confirmed_count = sum(
            1
            for track in obstacle_tracks
            if track["confirmed"]
        )

        cv2.putText(
            obstacle_frame,
            f"Tracks: {len(obstacle_tracks)}",
            (20, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            obstacle_frame,
            f"Confirmed: {confirmed_count}",
            (20, 190),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )


        # ====================================================
        # DEPTH VISUALIZATION
        # ====================================================

        depth_display = cv2.normalize(
            depth,
            None,
            0,
            255,
            cv2.NORM_MINMAX
        )

        depth_display = depth_display.astype(
            "uint8"
        )


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

        cv2.imshow(
            "Occupancy Grid Map",
            map_image
        )


        # ====================================================
        # KEYBOARD
        # ====================================================

        key = cv2.waitKey(1) & 0xFF


        # ----------------------------------------------------
        # Quit
        # ----------------------------------------------------

        if key == ord("q"):
            break


        # ====================================================
        # ROBOT MOVEMENT
        # ====================================================

        handle_keyboard(
            ugv
        )


        # ====================================================
        # PHYSICS
        # ====================================================

        p.stepSimulation()


# ============================================================
# CLEANUP
# ============================================================

finally:

    cv2.destroyAllWindows()

    close_simulation()

    trajectory.show()