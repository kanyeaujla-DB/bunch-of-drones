# sim/gazebo — rendering + simulated overhead camera (owner: gazebo)

Gazebo is NOT the physics here (the Pluto SDF has no motor/controller plugins).
Physics = `sim/plant` firmware emulator. Gazebo only:
1. renders a 3 x 3 m arena with Pluto models wearing ArUco textures (`models/`, `worlds/arena_3x3.sdf`);
2. provides an overhead camera sensor for the localization team;
3. `pose_bridge.py` moves each Gazebo model to the emulator's true pose
   (read from `SimWorld.truth()`), every frame.

Target: Gazebo Fortress + ROS 2 Humble (Ubuntu 22.04).
