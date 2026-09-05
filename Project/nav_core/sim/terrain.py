import pybullet as p
import numpy as np


def create_terrain():

    size = 100
    resolution = 100

    heights = np.zeros((resolution, resolution))

    # Create small terrain variations
    for x in range(resolution):
        for y in range(resolution):

            dx = (x - resolution / 2) / resolution
            dy = (y - resolution / 2) / resolution

            # gentle hills
            heights[x, y] = (
                0.15 * np.sin(dx * 15)
                + 0.10 * np.cos(dy * 12)
            )

    heights = heights.flatten()

    terrain_shape = p.createCollisionShape(
        shapeType=p.GEOM_HEIGHTFIELD,
        meshScale=[
            size / resolution,
            size / resolution,
            1
        ],
        heightfieldData=heights,
        numHeightfieldRows=resolution,
        numHeightfieldColumns=resolution
    )

    terrain = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=terrain_shape
    )

    return terrain