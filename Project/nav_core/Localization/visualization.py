# For example, the camera might struggle because of:

# darkness
# motion blur
# lack of visual features

# while wheel odometry can still provide movement information.

# Conversely, wheel odometry can accumulate drift, while visual information can help correct it.



import pybullet as p
import numpy as np


class Localization:

    def __init__(self, ugv):

        self.ugv = ugv

        # Estimated pose
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

    def get_ground_truth(self):

        position, orientation = p.getBasePositionAndOrientation(
            self.ugv
        )

        x = position[0]
        y = position[1]

        _, _, yaw = p.getEulerFromQuaternion(
            orientation
        )

        return x, y, yaw

    def update(self):

        self.x, self.y, self.heading = self.get_ground_truth()

        return self.x, self.y, self.heading