# Gazebo simulation

This branch adds a Gazebo world that approximates the semantic map and places
one ArUco sign for each stand declared in `config/semantic_map.yaml`.

## Regenerate marker textures

Run this after changing any `marker_id` in the semantic map:

```bash
rosrun event_guide_robot generate_gazebo_markers.py
```

From the repository without sourcing a catkin workspace:

```bash
python3 catkin_ws/src/event_guide_robot/scripts/generate_gazebo_markers.py
```

The generated PNGs use OpenCV's `DICT_4X4_50`, matching the default dictionary
used by `scripts/vision_detector_node.py`.

## Launch only the simulated world

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch event_guide_robot gazebo_event_world.launch
```

## Launch Gazebo, navigation, and guide nodes

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch event_guide_robot gazebo_navigation_with_guide.launch
```

The integrated launch uses:

- map: `maps/map.yaml`
- Gazebo world: `worlds/event_guide_world.world`
- camera topic: `/camera/rgb/image_raw`
- ArUco marker size: the existing `guide_system.launch` default

Send a command as usual, for example:

```bash
rostopic pub /guide/command std_msgs/String "data: 'qualcomm'"
```
