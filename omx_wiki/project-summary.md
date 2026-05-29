# Event Guide Robot project summary

Tags: ros1, turtlebot3, mobile-robotics, event-guide
Category: architecture

## Project idea

Build a ROS 1 project for a TurtleBot3 Waffle/Waffle Pi where the robot acts as a guide for an event such as MWC. The user asks for a destination, stand, or label. The robot navigates to the general semantic zone and then searches locally with the camera for a specific sign or marker.

## Assignment constraints

The final project statement asks for TurtleBot3 Waffle use, mobile navigation/control behavior, and at least these topics:

- `/cmd_vel`
- `/images` or equivalent camera topic
- `/scan`

The final presentation should include project description, developed solution, videos, and live demo in simulation or real robot.

## Current implementation status

The ROS package now exists at:

```text
catkin_ws/src/event_guide_robot/
```

Implemented nodes:

- `semantic_planner_node.py`: `/guide/command` -> `/guide/plan` JSON.
- `navigation_manager_node.py`: `/guide/plan` -> `/move_base` action goal.
- `local_search_manager_node.py`: waits for `NAVIGATION_SUCCEEDED`, then rotates on `/cmd_vel`, listens to `/vision/detections`, and stops safely on success/timeout.
- `vision_detector_node.py`: ArUco-based JSON detections when `cv2.aruco` is available; ArUco is the fixed MVP vision method.

Current semantic map:

- 4 measured zones from `puntos.txt`.
- 2 MWC-style stands per zone.
- numeric `marker_id` values that are the real ArUco IDs to print for each stand.

Local verification:

```text
26 pytest tests passed
Python py_compile OK
XML/YAML parse OK
```

Robot-real validation update: the package was copied into the original `~/catkin_ws/src` on a ROS 1 machine. With `/use_sim_time=false`, the real map loads, RViz shows the map, `move_base` navigation works against the real map, and the semantic planner resolves commands such as `qualcomm`. Camera/ArUco runtime validation is still pending.

## Architecture

```text
/guide/command
      |
      v
semantic_planner_node
      |
      v
navigation_manager_node ---> move_base ---> /cmd_vel
      |                       ^
      |                       |
      |              map_server + amcl + /scan
      v
local_search_manager_node
      |
      v
vision_detector_node <--- /camera/image
```

## Recommended implementation order

1. Verify navigation stack and map in RViz. **Done on real robot after setting `/use_sim_time=false`.**
2. Create `semantic_map.yaml` with zones and labels. **Done.**
3. Implement command parser / semantic planner. **Done.**
4. Integrate `move_base` with an action client. **Code done and real-robot navigation observed; camera pending.**
5. Implement local visual search by rotating. **Code done, runtime pending.**
6. Implement ArUco vision detections. **Code done; camera/ArUco runtime pending.**
7. Optional future vision extensions such as OCR/AprilTag/logos are out of MVP scope.

## Runtime notes

- For robot-real testing, set `rosparam set /use_sim_time false` before launching navigation. If it remains `true` from a Gazebo session, `/map` and `/map_metadata` can appear to hang and RViz may not display the map correctly even though `map_server` loaded the image.
- Use `event_guide_robot/navigation_with_guide.launch` when possible: it sets a full-turn `yaw_goal_tolerance` so semantic goals do not force final rotation toward the map center in tight spaces.
- Gazebo simulation needs a world that matches the 2D map. Using `turtlebot3_world` with the project map causes `/scan` to describe Gazebo walls, not the project map.
- Camera/ArUco validation has not yet been completed on the real robot.

## Future ideas

- Optional user-facing demo interface: start with a small Python CLI that lists stands, publishes `/guide/command`, and displays `/guide/state` plus `/guide/result`; if time remains, add a simple Tkinter UI.

## Main project documents

- `README.md`: complete architecture and project plan.
- `AGENTS.md`: instructions for future coding agents.
- `docs/superpowers/plans/2026-05-24-event-guide-robot.md`: implementation plan.
- `docs/semantic-labeling-guide.md`: how to create semantic labels once the map exists.
- `omx_wiki/semantic-labeling.md`: persistent wiki page for semantic labeling.
