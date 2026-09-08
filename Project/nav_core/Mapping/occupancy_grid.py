import numpy as np
import cv2


class OccupancyGrid:

    def __init__(
        self,
        map_size=20.0,
        resolution=0.1
    ):
        """
        map_size:
            Total map size in meters.
            Example: 20 means 20m x 20m.

        resolution:
            Size of each grid cell in meters.
            Example: 0.1 means each cell represents 10cm x 10cm.
        """

        self.map_size = map_size
        self.resolution = resolution

        # Number of cells in one dimension
        self.grid_size = int(map_size / resolution)

        # Occupancy grid
        #
        # 0   = unknown
        # 100 = obstacle
        #
        self.grid = np.zeros(
            (self.grid_size, self.grid_size),
            dtype=np.uint8
        )

        # Put the world origin (0,0) at the center of the map
        self.origin_x = -map_size / 2
        self.origin_y = -map_size / 2

    # ---------------------------------------------------------
    # Convert world coordinates -> grid coordinates
    # ---------------------------------------------------------

    def world_to_grid(self, x, y):

        grid_x = int(
            (x - self.origin_x) / self.resolution
        )  

        grid_y = int(
            (y - self.origin_y) / self.resolution
        )


        # Check whether point is inside map
        if (
            grid_x < 0
            or grid_x >= self.grid_size
            or grid_y < 0
            or grid_y >= self.grid_size
        ):
            return None

        return grid_x, grid_y

    # ---------------------------------------------------------
    # Convert grid coordinates -> world coordinates
    # ---------------------------------------------------------

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

    # ---------------------------------------------------------
    # Mark one obstacle
    # ---------------------------------------------------------

    def mark_obstacle(self, x, y):

        cell = self.world_to_grid(x, y)

        if cell is None:
            return

        grid_x, grid_y = cell

        self.grid[
            grid_y,
            grid_x
        ] = 100

    # ---------------------------------------------------------
    # Mark obstacle with a radius
    # ---------------------------------------------------------

    def mark_obstacle_area(
        self,
        x,
        y,
        radius=0.3
    ):

        cell = self.world_to_grid(x, y)

        if cell is None:
            return

        grid_x, grid_y = cell

        radius_cells = int(
            radius / self.resolution
        )

        cv2.circle(
            self.grid,
            (grid_x, grid_y),
            radius_cells,
            100,
            -1
        )

    # ---------------------------------------------------------
    # Add obstacles detected by perception
    # ---------------------------------------------------------

    def update_from_obstacles(
        self,
        obstacles
    ):
        """
        obstacles should contain world coordinates.

        Example:

        obstacles = [
            (3.0, 0.0),
            (5.0, 2.0),
            (7.0, -2.0)
        ]
        """

        for obstacle in obstacles:

            x, y = obstacle

            self.mark_obstacle_area(
                x,
                y,
                radius=0.3
            )

    # ---------------------------------------------------------
    # Mark robot position
    # ---------------------------------------------------------

    def get_robot_cell(
        self,
        x,
        y
    ):

        return self.world_to_grid(
            x,
            y
        )

        # ---------------------------------------------------------
    # Convert camera pixel + depth to world coordinates
    # -------------------------------------------------------
    #     # Camera
    #   ↓
    # pixel + depth
    #   ↓
    # camera coordinates
    #   ↓
    # UGV coordinates
    #   ↓
    # world coordinates

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
        camera_height=0.4,
        camera_forward_offset=0.8
    ):
        """
        Convert an obstacle detected in the camera image
        into approximate world coordinates.

        pixel_x, pixel_y:
            Obstacle pixel coordinates.

        depth:
            Depth value at that pixel in meters.

        robot_x, robot_y:
            Current UGV position.

        robot_heading:
            Current UGV heading in radians.
        """

        # -----------------------------------------------------
        # Camera parameters
        # -----------------------------------------------------

        cx = image_width / 2
        cy = image_height / 2

        # PyBullet FOV is treated as vertical FOV here
        fy = (
            image_height / 2
        ) / np.tan(
            np.radians(fov / 2)
        )

        fx = fy * (
            image_width / image_height
        )

        # -----------------------------------------------------
        # Pixel -> camera coordinates
        # -----------------------------------------------------

        # Horizontal displacement
        camera_right = (
            (pixel_x - cx)
            * depth
            / fx
        )

        # Vertical displacement
        camera_vertical = (
            (pixel_y - cy)
            * depth
            / fy
        )

        # Forward distance
        camera_forward = depth

        # -----------------------------------------------------
        # Camera -> robot coordinates
        #
        # Robot coordinate system:
        #
        #       +X = forward
        #       +Y = right
        #       +Z = up
        # -----------------------------------------------------

        robot_forward = (
            camera_forward
            + camera_forward_offset
        )

        robot_right = camera_right

        # -----------------------------------------------------
        # Robot -> world coordinates
        # -----------------------------------------------------

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

    # ---------------------------------------------------------
    # Create visualization of map
    # ---------------------------------------------------------

    def get_map_image(
        self,
        robot_x=None,
        robot_y=None,
        robot_heading=None
    ):

        # Convert occupancy grid into display image
        #
        # Unknown/free = white
        # Obstacles = black
        #
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

        # Convert grayscale to BGR
        image = cv2.cvtColor(
            image,
            cv2.COLOR_GRAY2BGR
        )

        # -----------------------------------------------------
        # Draw robot
        # -----------------------------------------------------

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

                # OpenCV image coordinates have Y downward,
                # so flip Y for display.
                display_y = (
                    self.grid_size - 1 - grid_y
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

                # Draw heading
                if robot_heading is not None:

                    arrow_length = 20

                    end_x = int(
                        grid_x
                        + arrow_length
                        * np.cos(robot_heading)
                        / self.resolution
                    )

                    end_y = int(
                        display_y
                        - arrow_length
                        * np.sin(robot_heading)
                        / self.resolution
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

        # -----------------------------------------------------
        # Resize map for easier viewing
        # -----------------------------------------------------

        display_size = 600

        image = cv2.resize(
            image,
            (
                display_size,
                display_size
            ),
            interpolation=cv2.INTER_NEAREST
        )

        # Add title
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

    # ---------------------------------------------------------
    # Reset map
    # ---------------------------------------------------------

    def clear(self):

        self.grid.fill(0)