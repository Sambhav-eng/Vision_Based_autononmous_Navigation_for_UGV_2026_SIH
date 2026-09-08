import heapq
import math


class AStarPlanner:

    def __init__(self, occupancy_grid):

        self.grid = occupancy_grid.grid

        self.grid_size = occupancy_grid.grid_size

    # ========================================================
    # HEURISTIC
    # ========================================================

    def heuristic(self, a, b):

        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])

        return math.sqrt(
            dx * dx +
            dy * dy
        )

    # ========================================================
    # CHECK WHETHER CELL IS VALID
    # ========================================================

    def is_free(self, x, y):

        # Outside map
        if x < 0 or x >= self.grid_size:
            return False

        if y < 0 or y >= self.grid_size:
            return False

        # Occupied cell
        if self.grid[y, x] == 100:
            return False

        return True

    # ========================================================
    # GET NEIGHBOURS
    # ========================================================

    def get_neighbors(self, node):

        x, y = node

        directions = [
            (-1,  0),
            ( 1,  0),
            ( 0, -1),
            ( 0,  1),

            # Diagonal movement
            (-1, -1),
            (-1,  1),
            ( 1, -1),
            ( 1,  1)
        ]

        neighbors = []

        for dx, dy in directions:

            nx = x + dx
            ny = y + dy

            if self.is_free(nx, ny):

                neighbors.append(
                    (nx, ny)
                )

        return neighbors

    # ========================================================
    # PATH PLANNING
    # ========================================================

    def plan(self, start, goal):

        # ----------------------------------------------------
        # Check start and goal
        # ----------------------------------------------------

        if not self.is_free(
            start[0],
            start[1]
        ):
            print("Start position is occupied.")
            return []

        if not self.is_free(
            goal[0],
            goal[1]
        ):
            print("Goal position is occupied.")
            return []


        # ----------------------------------------------------
        # Priority queue
        #
        # (f_cost, node)
        # ----------------------------------------------------

        open_set = []

        heapq.heappush(
            open_set,
            (
                0,
                start
            )
        )


        # ----------------------------------------------------
        # Cost from start
        # ----------------------------------------------------

        g_cost = {
            start: 0
        }


        # ----------------------------------------------------
        # Parent of each node
        # ----------------------------------------------------

        came_from = {}


        # ----------------------------------------------------
        # A* search
        # ----------------------------------------------------

        while open_set:

            current_f, current = heapq.heappop(
                open_set
            )


            # ------------------------------------------------
            # Goal reached
            # ------------------------------------------------

            if current == goal:

                return self.reconstruct_path(
                    came_from,
                    current
                )


            # ------------------------------------------------
            # Explore neighbours
            # ------------------------------------------------

            for neighbor in self.get_neighbors(
                current
            ):

                # Cost of moving
                dx = neighbor[0] - current[0]
                dy = neighbor[1] - current[1]

                if dx != 0 and dy != 0:
                    movement_cost = math.sqrt(2)
                else:
                    movement_cost = 1


                # New cost
                new_g_cost = (
                    g_cost[current]
                    +
                    movement_cost
                )


                # ------------------------------------------------
                # Is this a better path?
                # ------------------------------------------------

                if (
                    neighbor not in g_cost
                    or
                    new_g_cost
                    < g_cost[neighbor]
                ):

                    g_cost[neighbor] = new_g_cost

                    h_cost = self.heuristic(
                        neighbor,
                        goal
                    )

                    f_cost = (
                        new_g_cost
                        +
                        h_cost
                    )


                    # Save parent
                    came_from[
                        neighbor
                    ] = current


                    # Add to priority queue
                    heapq.heappush(
                        open_set,
                        (
                            f_cost,
                            neighbor
                        )
                    )


        # ----------------------------------------------------
        # No path found
        # ----------------------------------------------------

        print("No path found.")

        return []

    # ========================================================
    # RECONSTRUCT PATH
    # ========================================================

    def reconstruct_path(
        self,
        came_from,
        current
    ):

        path = [
            current
        ]

        while current in came_from:

            current = came_from[
                current
            ]

            path.append(
                current
            )

        # Reverse
        path.reverse()

        return path