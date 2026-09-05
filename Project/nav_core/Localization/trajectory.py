#-----------------------It will show the graph of the trajectory of the UGV based on visual odometry and ground truth data-----------------------#

import matplotlib.pyplot as plt


class TrajectoryTracker:

    def __init__(self):

        self.vo_x = []
        self.vo_y = []

        self.gt_x = []
        self.gt_y = []

    def update(
        self,
        vo_x,
        vo_y,
        gt_x,
        gt_y
    ):

        self.vo_x.append(vo_x)
        self.vo_y.append(vo_y)

        self.gt_x.append(gt_x)
        self.gt_y.append(gt_y)

    def show(self):

        if len(self.vo_x) == 0:
            return

        plt.figure(figsize=(10, 7))

        # Visual Odometry trajectory
        plt.plot(
            self.vo_x,
            self.vo_y,
            label="Visual Odometry"
        )

        # Ground Truth trajectory
        plt.plot(
            self.gt_x,
            self.gt_y,
            label="Ground Truth"
        )

        # Starting point
        plt.scatter(
            self.gt_x[0],
            self.gt_y[0],
            label="Start"
        )

        plt.xlabel("X Position")
        plt.ylabel("Y Position")

        plt.title(
            "UGV Localization: "
            "Visual Odometry vs Ground Truth"
        )

        plt.legend()
        plt.grid()

        plt.axis("equal")

        plt.show()