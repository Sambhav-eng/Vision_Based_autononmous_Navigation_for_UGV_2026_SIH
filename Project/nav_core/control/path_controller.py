import math


class PathController:

    def __init__(
        self,
        waypoint_threshold=0.3,
        max_speed=2.0,
        max_turn=2.0
    ):

        self.waypoint_threshold = waypoint_threshold
        self.max_speed = max_speed
        self.max_turn = max_turn

        self.current_waypoint = 0


    def reset(self):

        self.current_waypoint = 0


    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi

        while angle < -math.pi:
            angle += 2 * math.pi

        return angle


    def update(
        self,
        robot_x,
        robot_y,
        robot_heading,
        path
    ):

        # No path
        if len(path) == 0:
            return 0, 0, False


        # Finished path
        if self.current_waypoint >= len(path):
            return 0, 0, True


        # Current waypoint
        waypoint_x, waypoint_y = path[
            self.current_waypoint
        ]


        # ----------------------------------------------------
        # Distance to waypoint
        # ----------------------------------------------------

        dx = waypoint_x - robot_x
        dy = waypoint_y - robot_y

        distance = math.sqrt(
            dx * dx + dy * dy
        )


        # ----------------------------------------------------
        # If close enough → next waypoint
        # ----------------------------------------------------

        if distance < self.waypoint_threshold:

            self.current_waypoint += 1

            if self.current_waypoint >= len(path):

                return 0, 0, True

            waypoint_x, waypoint_y = path[
                self.current_waypoint
            ]

            dx = waypoint_x - robot_x
            dy = waypoint_y - robot_y


        # ----------------------------------------------------
        # Desired heading
        # ----------------------------------------------------

        desired_heading = math.atan2(
            dy,
            dx
        )


        # ----------------------------------------------------
        # Heading error
        # ----------------------------------------------------

        heading_error = self.normalize_angle(
            desired_heading - robot_heading
        )


        # ----------------------------------------------------
        # Steering
        # ----------------------------------------------------

        turn = heading_error


        # Limit turning
        turn = max(
            -self.max_turn,
            min(
                self.max_turn,
                turn
            )
        )


        # ----------------------------------------------------
        # Speed
        # ----------------------------------------------------

        speed = self.max_speed


        # If robot is facing far away from waypoint,
        # slow down.

        if abs(heading_error) > 1.0:

            speed = 0.5

        elif abs(heading_error) > 0.5:

            speed = 1.0


        return speed, turn, False