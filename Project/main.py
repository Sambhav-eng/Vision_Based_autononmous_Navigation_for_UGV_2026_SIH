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
# Path Planning (A*)
#   ↓
# Controller
#   ↓
# UGV
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

from nav_core.planning.astar import AStarPlanner

# Controller
from nav_core.control.path_controller import PathController


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

planner = AStarPlanner(mapping)

controller = PathController()


# ============================================================
# GOAL POSITION
#
# Goal is defined in WORLD coordinates.
# ============================================================

GOAL_X = 8.0
GOAL_Y = 0.0


# ============================================================
# TEMPORAL OBSTACLE TRACKING
# ============================================================

obstacle_tracks = []

TRACK_DISTANCE_THRESHOLD = 0.7

SMOOTHING_ALPHA = 0.2

MIN_OBSERVATIONS = 5


# ============================================================
# UPDATE OBSTACLE TRACKS
# ============================================================

def update_obstacle_tracks(detections):

    for x, y in detections:

        best_track = None
        best_distance = float("inf")


        # ----------------------------------------------------
        # Find closest unconfirmed track
        # ----------------------------------------------------

        for track in obstacle_tracks:

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


        # ====================================================
        # EXISTING TRACK
        # ====================================================

        if best_track is not None:

            best_track["x"] = (
                (1 - SMOOTHING_ALPHA)
                * best_track["x"]
                +
                SMOOTHING_ALPHA
                * x
            )


            best_track["y"] = (
                (1 - SMOOTHING_ALPHA)
                * best_track["y"]
                +
                SMOOTHING_ALPHA
                * y
            )


            best_track["observations"] += 1


        # ====================================================
        # NEW TRACK
        # ====================================================

        else:

            obstacle_tracks.append(
                {
                    "x": x,
                    "y": y,
                    "observations": 1,
                    "confirmed": False
                }
            )


# ============================================================
# CONFIRM STABLE OBSTACLES
# ============================================================

def commit_stable_obstacles():

    for track in obstacle_tracks:

        # Already inserted into map
        if track["confirmed"]:
            continue


        # Wait for enough observations
        if track["observations"] >= MIN_OBSERVATIONS:

            track["confirmed"] = True


            # Add obstacle to persistent map
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
print("A* Path Planning    : ON")
print("Path Controller     : ON")
print("AUTONOMOUS MODE     : ON")
print("------------------------------------------")
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

        obstacle_frame, obstacle_mask, obstacles = (
            detect_obstacles(frame)
        )


        # ====================================================
        # FREE SPACE DETECTION
        # ====================================================

        free_space_frame, free_space_mask = (
            detect_free_space(frame)
        )


        # ====================================================
        # MAPPING
        # ====================================================

        world_obstacles = []


        for obstacle in obstacles:

            # ------------------------------------------------
            # Bottom-center of detected obstacle
            # ------------------------------------------------

            pixel_x, pixel_y = obstacle[
                "bottom_center"
            ]


            # ------------------------------------------------
            # Check pixel bounds
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
            # Pixel + depth → world coordinates
            # ------------------------------------------------

            world_x, world_y = mapping.obstacle_to_world(

                pixel_x,
                pixel_y,

                obstacle_depth,

                gt_x,
                gt_y,
                gt_heading
            )


            world_obstacles.append(
                (
                    world_x,
                    world_y
                )
            )


        # ====================================================
        # TEMPORAL FILTERING
        # ====================================================

        update_obstacle_tracks(
            world_obstacles
        )


        # ====================================================
        # CONFIRM STABLE OBSTACLES
        # ====================================================

        commit_stable_obstacles()


        # ====================================================
        # A* PATH PLANNING
        # ====================================================

        # Robot WORLD → GRID

        start_cell = mapping.world_to_grid(
            gt_x,
            gt_y
        )


        # Goal WORLD → GRID

        goal_cell = mapping.world_to_grid(
            GOAL_X,
            GOAL_Y
        )


        path = []


        # ====================================================
        # RUN A*
        # ====================================================

        if (
            start_cell is not None
            and goal_cell is not None
        ):

            # Give planner latest map

            planner.grid = mapping.grid


            path = planner.plan(
                start_cell,
                goal_cell
            )


        # ====================================================
        # CONVERT A* GRID PATH → WORLD PATH
        # ====================================================

        world_path = []


        for cell in path:

            grid_x, grid_y = cell


            world_x, world_y = (
                mapping.grid_to_world(
                    grid_x,
                    grid_y
                )
            )


            world_path.append(
                (
                    world_x,
                    world_y
                )
            )


        # ====================================================
        # AUTONOMOUS CONTROLLER
        # ====================================================

        speed = 0
        turn = 0


        if len(world_path) > 0:

            speed, turn, finished = (
                controller.update(

                    gt_x,
                    gt_y,

                    gt_heading,

                    world_path
                )
            )


            # ------------------------------------------------
            # GOAL REACHED
            # ------------------------------------------------

            if finished:

                speed = 0
                turn = 0

                print(
                    "================================"
                )

                print(
                    "        GOAL REACHED!"
                )

                print(
                    "================================"
                )


        else:

            # No path available
            speed = 0
            turn = 0


        # ====================================================
        # MOVE UGV AUTOMATICALLY
        # ====================================================

        move_robot(
            ugv,
            speed,
            turn
        )


        # ====================================================
        # CREATE MAP IMAGE
        # ====================================================

        map_image = mapping.get_map_image(

            robot_x=gt_x,

            robot_y=gt_y,

            robot_heading=gt_heading
        )


        # ====================================================
        # DRAW A* PATH
        # ====================================================

        if len(path) > 0:

            for i in range(
                len(path) - 1
            ):

                x1, y1 = path[i]

                x2, y2 = path[i + 1]


                # --------------------------------------------
                # GRID → DISPLAY
                # --------------------------------------------

                display_x1 = int(
                    x1
                    * 600
                    / mapping.grid_size
                )


                display_y1 = int(
                    (
                        mapping.grid_size
                        - 1
                        - y1
                    )
                    * 600
                    / mapping.grid_size
                )


                display_x2 = int(
                    x2
                    * 600
                    / mapping.grid_size
                )


                display_y2 = int(
                    (
                        mapping.grid_size
                        - 1
                        - y2
                    )
                    * 600
                    / mapping.grid_size
                )


                # --------------------------------------------
                # Draw path
                # --------------------------------------------

                cv2.line(

                    map_image,

                    (
                        display_x1,
                        display_y1
                    ),

                    (
                        display_x2,
                        display_y2
                    ),

                    (0, 255, 0),

                    2
                )


        # ====================================================
        # DRAW GOAL
        # ====================================================

        goal_display = mapping.world_to_grid(
            GOAL_X,
            GOAL_Y
        )


        if goal_display is not None:

            goal_x, goal_y = goal_display


            display_goal_x = int(
                goal_x
                * 600
                / mapping.grid_size
            )


            display_goal_y = int(
                (
                    mapping.grid_size
                    - 1
                    - goal_y
                )
                * 600
                / mapping.grid_size
            )


            cv2.circle(

                map_image,

                (
                    display_goal_x,
                    display_goal_y
                ),

                8,

                (0, 0, 255),

                -1
            )


            cv2.putText(

                map_image,

                "GOAL",

                (
                    display_goal_x + 10,
                    display_goal_y
                ),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.5,

                (0, 0, 255),

                2
            )


        # ====================================================
        # LOCALIZATION INFORMATION
        # ====================================================

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


        # ====================================================
        # GROUND TRUTH
        # ====================================================

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
        # A* INFORMATION
        # ====================================================

        cv2.putText(

            obstacle_frame,

            f"Path cells: {len(path)}",

            (20, 220),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.7,

            (255, 255, 255),

            2
        )


        # ====================================================
        # CONTROLLER INFORMATION
        # ====================================================

        cv2.putText(

            obstacle_frame,

            f"Speed: {speed:.2f}",

            (20, 250),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.7,

            (0, 255, 0),

            2
        )


        cv2.putText(

            obstacle_frame,

            f"Turn: {turn:.2f}",

            (20, 280),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.7,

            (0, 255, 0),

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
        # QUIT
        # ====================================================

        key = cv2.waitKey(1) & 0xFF


        if key == ord("q"):

            break


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