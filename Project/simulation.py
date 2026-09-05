#-------------------------------A working simulation of a robot car , ugv ---------------------------------------------------------------------



import pybullet as p
import pybullet_data
import time
import os


#----------------------------------Adding terrain to the simulation-------------------------------------------------
# Terrain loading logic now lives in terrain_utils.py so camera.py and
# simulation.py can't drift out of sync with each other again.
from terrain_utils import get_terrain_texture_id, apply_ground_terrain, create_backdrop_wall




# ============================================================
# 1. START PYBULLET
# ============================================================

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)



plane_id = p.loadURDF("plane.urdf")

# Load the terrain image once (auto-resized if needed), reuse for ground + backdrop
project_dir = os.path.dirname(__file__)
terrain_texture_id = get_terrain_texture_id(project_dir)

# Make the ground the UGV drives on look like real terrain
apply_ground_terrain(plane_id, terrain_texture_id)

# Create distant backdrop wall using the same terrain texture
create_backdrop_wall(terrain_texture_id)

# Make it look like a real environment: shadows + a camera angle that
# actually frames the robot and the ground instead of the default top-down view
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
p.resetDebugVisualizerCamera(
    cameraDistance=6,
    cameraYaw=50,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.5]
)


# ============================================================
# 2. CREATE UGV
# ============================================================

# Body dimensions
body_length = 1.6
body_width = 1.0
body_height = 0.4

wheel_radius = 0.25
wheel_width = 0.18


# ------------------------------------------------------------
# Chassis
# ------------------------------------------------------------

chassis_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[
        body_length / 2,
        body_width / 2,
        body_height / 2
    ]
)

chassis_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[
        body_length / 2,
        body_width / 2,
        body_height / 2
    ]
)


# ------------------------------------------------------------
# Wheel collision/visual shapes
# ------------------------------------------------------------

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


# ============================================================
# 3. CREATE MULTIBODY
# ============================================================

# Four wheels:
#
#       FRONT
#
#       0       1
#
#
#       2       3
#
#       BACK
#
# 0 = Front Left
# 1 = Front Right
# 2 = Rear Left
# 3 = Rear Right


link_masses = [
    1,  # Front Left
    1,  # Front Right
    1,  # Rear Left
    1   # Rear Right
]


link_collision_shapes = [
    wheel_collision,
    wheel_collision,
    wheel_collision,
    wheel_collision
]


link_visual_shapes = [
    wheel_visual,
    wheel_visual,
    wheel_visual,
    wheel_visual
]


# Wheel positions relative to chassis

link_positions = [
    [0.55,  0.58, -0.25],
    [0.55, -0.58, -0.25],
    [-0.55,  0.58, -0.25],
    [-0.55, -0.58, -0.25]
]


# All wheel axles point along Y

link_orientations = [
    p.getQuaternionFromEuler([1.5708, 0, 0]),
    p.getQuaternionFromEuler([1.5708, 0, 0]),
    p.getQuaternionFromEuler([1.5708, 0, 0]),
    p.getQuaternionFromEuler([1.5708, 0, 0])
]


# Each wheel rotates around Y

link_inertial_positions = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0]
]


link_inertial_orientations = [
    [0, 0, 0, 1],
    [0, 0, 0, 1],
    [0, 0, 0, 1],
    [0, 0, 0, 1]
]


link_parent_indices = [
    0,
    0,
    0,
    0
]


link_joint_types = [
    p.JOINT_REVOLUTE,
    p.JOINT_REVOLUTE,
    p.JOINT_REVOLUTE,
    p.JOINT_REVOLUTE
]


link_joint_axes = [
    [0, 1, 0],
    [0, 1, 0],
    [0, 1, 0],
    [0, 1, 0]
]


ugv = p.createMultiBody(
    baseMass=20,
    baseCollisionShapeIndex=chassis_collision,
    baseVisualShapeIndex=chassis_visual,
    basePosition=[0, 0, 0.55],

    linkMasses=link_masses,
    linkCollisionShapeIndices=link_collision_shapes,
    linkVisualShapeIndices=link_visual_shapes,

    linkPositions=link_positions,
    linkOrientations=link_orientations,

    linkInertialFramePositions=link_inertial_positions,
    linkInertialFrameOrientations=link_inertial_orientations,

    linkParentIndices=link_parent_indices,
    linkJointTypes=link_joint_types,
    linkJointAxis=link_joint_axes
)


# ============================================================
# 4. CONTROL SETTINGS
# ============================================================

wheel_speed = 10

print()
print("======================================")
print("          UGV SIMULATION")
print("======================================")
print()
print("W -> Forward")
print("S -> Reverse")
print("A -> Turn Left")
print("D -> Turn Right")
print("SPACE -> Stop")
print("Q -> Quit")
print()
print("======================================")






# ============================================================
# 5. MAIN LOOP
# ============================================================

while p.isConnected():

    keys = p.getKeyboardEvents()


    # Default speeds
    left_speed = 0
    right_speed = 0


    # --------------------------------------------------------
    # FORWARD
    # --------------------------------------------------------

    if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:

        left_speed = wheel_speed
        right_speed = wheel_speed


    # --------------------------------------------------------
    # REVERSE
    # --------------------------------------------------------

    elif ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:

        left_speed = -wheel_speed
        right_speed = -wheel_speed


    # --------------------------------------------------------
    # TURN LEFT
    # --------------------------------------------------------

    elif ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:

        left_speed = 4
        right_speed = 10


    # --------------------------------------------------------
    # TURN RIGHT
    # --------------------------------------------------------

    elif ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:

        left_speed = 10
        right_speed = 4


    # --------------------------------------------------------
    # STOP
    # --------------------------------------------------------

    else:

        left_speed = 0
        right_speed = 0


    # ========================================================
    # APPLY WHEEL MOTOR CONTROL
    # ========================================================

    # Left wheels = joints 0 and 2
    # Right wheels = joints 1 and 3

    for joint in [0, 2]:

        p.setJointMotorControl2(
            bodyUniqueId=ugv,
            jointIndex=joint,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=left_speed,
            force=50
        )


    for joint in [1, 3]:

        p.setJointMotorControl2(
            bodyUniqueId=ugv,
            jointIndex=joint,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=right_speed,
            force=50
        )


    # ========================================================
    # STEP SIMULATION
    # ========================================================

    p.stepSimulation()

    time.sleep(1 / 240)


# ============================================================
# 6. CLOSE
# ============================================================

p.disconnect()