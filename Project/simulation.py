import pybullet as p
import pybullet_data
import time


# --------------------------------
# 1. Start PyBullet
# --------------------------------
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Gravity
p.setGravity(0, 0, -9.81)


# --------------------------------
# 2. Add Ground
# --------------------------------
planeId = p.loadURDF("plane.urdf")


# --------------------------------
# 3. Create UGV Body
# --------------------------------

# UGV dimensions
length = 2.0
width = 1.2
height = 0.4

# Position of UGV
start_position = [0, 0, 0.5]

# Collision shape
collision_shape = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[
        length / 2,
        width / 2,
        height / 2
    ]
)

# Visual shape
visual_shape = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[
        length / 2,
        width / 2,
        height / 2
    ]
)

# Create UGV
ugv = p.createMultiBody(
    baseMass=20,
    baseCollisionShapeIndex=collision_shape,
    baseVisualShapeIndex=visual_shape,
    basePosition=start_position
)


# --------------------------------
# 4. Create Wheels
# --------------------------------

wheel_radius = 0.3
wheel_width = 0.2

wheel_collision = p.createCollisionShape(
    p.GEOM_CYLINDER,
    radius=wheel_radius,
    height=wheel_width
)

wheel_visual = p.createVisualShape(
    p.GEOM_CYLINDER,
    radius=wheel_radius,
    length=wheel_width
)


# Wheel positions
wheel_positions = [
    [0.65, 0.65, 0],
    [0.65, -0.65, 0],
    [-0.65, 0.65, 0],
    [-0.65, -0.65, 0]
]


# --------------------------------
# 5. Attach Wheels
# --------------------------------

for position in wheel_positions:

    wheel = p.createMultiBody(
        baseMass=2,
        baseCollisionShapeIndex=wheel_collision,
        baseVisualShapeIndex=wheel_visual,
        basePosition=[
            start_position[0] + position[0],
            start_position[1] + position[1],
            start_position[2] - 0.3
        ]
    )

    # Create visual/physical connection
    p.createConstraint(
        ugv,
        -1,
        wheel,
        -1,
        p.JOINT_FIXED,
        [0, 0, 0],
        position,
        [0, 0, 0]
    )


# --------------------------------
# 6. Simulation Loop
# --------------------------------

print("UGV simulation started!")
print("Close the PyBullet window to stop.")

while p.isConnected():

    p.stepSimulation()

    time.sleep(1 / 240)


# --------------------------------
# 7. Disconnect
# --------------------------------

p.disconnect()