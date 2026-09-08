import numpy as np
import cv2


class OccupancyGrid:

    def __init__(
        self,
        map_size=20.0,
        resolution=0.1
    ):

        self.map_size = map_size
        self.resolution = resolution

        self.grid_size = int(
            map_size / resolution
        )

        # 0 = unknown/free
        # 100 = obstacle
        self.grid = np.zeros(
            (self.grid_size, self.grid_size),
            dtype=np.uint8
        )

        # World origin at center of map
        self.origin_x = -map_size / 2
        self.origin_y = -map_size / 2

        # -------------------------------------------------
        # Temporary obstacle tracks
        # -------------------------------------------------

        self.obstacle_tracks = []

        # Maximum distance at which two observations
        # are considered the same obstacle.
        self.track_distance_threshold = 0.7

        # Smoothing factor
        #
        # Smaller = smoother but slower
        # Larger  = reacts faster but noisier
        self.alpha = 0.2

        # Number of observations required before
        # committing an obstacle to the persistent map.
        self.min_observations = 3

    # =====================================================
    # WORLD → GRID
    # =====================================================

    def world_to_grid(self, x, y):

        grid_x = int(
            (x - self.origin_x)
            / self.resolution
        )

        grid_y = int(
            (y - self.origin_y)
            / self.resolution
        )

        if (
            grid_x < 0
            or grid_x >= self.grid_size
            or grid_y < 0
            or grid_y >= self.grid_size
        ):
            return None

        return grid_x, grid_y

    # =====================================================
    # GRID → WORLD
    # =====================================================

    def grid_to_world(self, grid_x, grid_y):

        x = (
            grid_x * self.resolution
            + self.origin_x
            + self.resolution / 2
        )

        y = (
            grid_y * self.resolution
            + self.origin_y
            + self.resolution / 2
        )

        return x, y

    # =====================================================
    # MARK ONE OBSTACLE
    # =====================================================

    def mark_obstacle(
        self,
        x,
        y
    ):

        cell = self.world_to_grid(
            x,
            y
        )

        if cell is None:
            return

        grid_x, grid_y = cell

        self.grid[
            grid_y,
            grid_x
        ] = 100

    # =====================================================
    # MARK OBSTACLE AREA
    # =====================================================

    def mark_obstacle_area(
        self,
        x,
        y,
        radius=0.3
    ):

        cell = self.world_to_grid(
            x,
            y
        )

        if cell is None:
            return

        grid_x, grid_y = cell

        radius_cells = max(
            1,
            int(
                radius
                / self.resolution
            )
        )

        cv2.circle(
            self.grid,
            (
                grid_x,
                grid_y
            ),
            radius_cells,
            100,
            -1
        )

    # =====================================================
    # OBSTACLE PIXEL → WORLD
    # =====================================================

    def obstacle_to_world(
        self,
        pixel_x,
        pixel_y,
        depth,
        robot_x,
        robot_y,
        robot_heading,
        image_width=640,
        image_height=480,
        fov=70,
        camera_forward_offset=0.8
    ):

        cx = image_width / 2
        cy = image_height / 2

        fy = (
            image_height / 2
            / np.tan(
                np.radians(fov / 2)
            )
        )

        fx = (
            fy
            * image_width
            / image_height
        )

        # Camera coordinates
        camera_right = (
            (pixel_x - cx)
            * depth
            / fx
        )

        camera_forward = depth

        # Camera is mounted forward of robot center
        robot_forward = (
            camera_forward
            + camera_forward_offset
        )

        robot_right = camera_right

        # Camera-relative → world
        world_x = (
            robot_x
            + robot_forward
            * np.cos(robot_heading)
            - robot_right
            * np.sin(robot_heading)
        )

        world_y = (
            robot_y
            + robot_forward
            * np.sin(robot_heading)
            + robot_right
            * np.cos(robot_heading)
        )

        return world_x, world_y

    # =====================================================
    # UPDATE TEMPORARY OBSTACLE TRACKS
    # =====================================================

    def update_obstacle_tracks(
        self,
        detected_obstacles
    ):
        """
        detected_obstacles:
            list of (world_x, world_y)

        These observations are NOT immediately written
        into the persistent occupancy grid.
        """

        for x, y in detected_obstacles:

            best_track = None
            best_distance = float("inf")

            # ---------------------------------------------
            # Find nearest existing track
            # ---------------------------------------------

            for track in self.obstacle_tracks:

                distance = np.sqrt(
                    (x - track["x"]) ** 2
                    +
                    (y - track["y"]) ** 2
                )

                if (
                    distance
                    < self.track_distance_threshold
                    and distance
                    < best_distance
                ):

                    best_track = track
                    best_distance = distance

            # ---------------------------------------------
            # Existing obstacle
            # ---------------------------------------------

            if best_track is not None:

                # Exponential moving average
                best_track["x"] = (
                    (1 - self.alpha)
                    * best_track["x"]
                    +
                    self.alpha
                    * x
                )

                best_track["y"] = (
                    (1 - self.alpha)
                    * best_track["y"]
                    +
                    self.alpha
                    * y
                )

                best_track["observations"] += 1

            # ---------------------------------------------
            # New obstacle
            # ---------------------------------------------

            else:

                self.obstacle_tracks.append(
                    {
                        "x": x,
                        "y": y,
                        "observations": 1
                    }
                )

    # =====================================================
    # COMMIT STABLE TRACKS TO MAP
    # =====================================================

    def commit_stable_obstacles(
        self,
        radius=0.3
    ):

        for track in self.obstacle_tracks:

            if (
                track["observations"]
                >= self.min_observations
            ):

                self.mark_obstacle_area(
                    track["x"],
                    track["y"],
                    radius
                )

    # =====================================================
    # UPDATE FROM WORLD OBSTACLES
    # =====================================================

    def update_from_obstacles(
        self,
        obstacles
    ):

        self.update_obstacle_tracks(
            obstacles
        )

        self.commit_stable_obstacles()

    # =====================================================
    # ROBOT CELL
    # =====================================================

    def get_robot_cell(
        self,
        x,
        y
    ):

        return self.world_to_grid(
            x,
            y
        )

    # =====================================================
    # MAP IMAGE
    # =====================================================

    def get_map_image(
        self,
        robot_x=None,
        robot_y=None,
        robot_heading=None
    ):

        image = np.ones(
            (
                self.grid_size,
                self.grid_size
            ),
            dtype=np.uint8
        ) * 255

        image[
            self.grid == 100
        ] = 0

        image = cv2.cvtColor(
            image,
            cv2.COLOR_GRAY2BGR
        )

        # ---------------------------------------------
        # Robot
        # ---------------------------------------------

        if (
            robot_x is not None
            and robot_y is not None
        ):

            cell = self.world_to_grid(
                robot_x,
                robot_y
            )

            if cell is not None:

                grid_x, grid_y = cell

                display_y = (
                    self.grid_size
                    - 1
                    - grid_y
                )

                cv2.circle(
                    image,
                    (
                        grid_x,
                        display_y
                    ),
                    5,
                    (0, 0, 255),
                    -1
                )

                # -------------------------------------
                # Robot heading
                # -------------------------------------

                if robot_heading is not None:

                    arrow_length = 2.0

                    end_x = int(
                        grid_x
                        + (
                            arrow_length
                            * np.cos(
                                robot_heading
                            )
                            / self.resolution
                        )
                    )

                    end_y = int(
                        display_y
                        - (
                            arrow_length
                            * np.sin(
                                robot_heading
                            )
                            / self.resolution
                        )
                    )

                    cv2.arrowedLine(
                        image,
                        (
                            grid_x,
                            display_y
                        ),
                        (
                            end_x,
                            end_y
                        ),
                        (255, 0, 0),
                        2,
                        tipLength=0.3
                    )

        # ---------------------------------------------
        # Resize
        # ---------------------------------------------

        image = cv2.resize(
            image,
            (
                600,
                600
            ),
            interpolation=cv2.INTER_NEAREST
        )

        cv2.putText(
            image,
            "Occupancy Grid Map",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 0),
            2
        )

        return image

    # =====================================================
    # CLEAR MAP
    # =====================================================

    def clear(self):

        self.grid.fill(0)

        self.obstacle_tracks.clear()