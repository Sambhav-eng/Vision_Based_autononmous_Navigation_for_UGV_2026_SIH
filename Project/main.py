#-----------------------------------Brain ---------

# main.py
#    │
#    ├── create simulation
#    │
#    ├── get camera image
#    │
#    ├── detect obstacles
#    │
#    ├── detect free space
#    │
#    ├── SLAM
#    │
#    ├── planning
#    │
#    └── control robot

from camera import create_world, get_camera_image
from perception.obstacle_detection import detect_obstacles
from perception.free_space import detect_free_space

ugv = create_world()

while True:

    frame = get_camera_image(ugv)

    obstacles = detect_obstacles(frame)

    free_space = detect_free_space(frame)