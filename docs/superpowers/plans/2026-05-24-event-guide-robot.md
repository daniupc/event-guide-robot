# Event Guide Robot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS 1 TurtleBot3 Waffle event-guide robot that navigates to semantic zones and visually finds a requested sign or marker.

**Architecture:** Use `move_base`, `map_server`, `amcl`, `/scan`, and `/cmd_vel` for navigation, with a separate semantic YAML layer for event zones. Coordinate behavior through small Python ROS nodes: semantic planner, navigation manager, local search manager, and vision detector.

**Tech Stack:** ROS 1, TurtleBot3 Waffle/Waffle Pi, Python ROS nodes, YAML configuration, OpenCV, ArUco/AprilTag MVP, optional OCR.

---

## File structure

- Created `catkin_ws/src/event_guide_robot/`: ROS package root.
- Created `catkin_ws/src/event_guide_robot/config/semantic_map.yaml`: semantic zones, aliases, goals, labels.
- Created `catkin_ws/src/event_guide_robot/config/search_params.yaml`: rotation speed, timeouts, confirmation frames.
- Created `catkin_ws/src/event_guide_robot/scripts/semantic_planner_node.py`: command-to-zone resolver.
- Created `catkin_ws/src/event_guide_robot/scripts/navigation_manager_node.py`: `move_base` action client.
- Created `catkin_ws/src/event_guide_robot/scripts/local_search_manager_node.py`: zone search state machine.
- Created `catkin_ws/src/event_guide_robot/scripts/vision_detector_node.py`: visual detector.
- Created `catkin_ws/src/event_guide_robot/launch/guide_system.launch`: launches project nodes.
- Created `catkin_ws/src/event_guide_robot/launch/navigation_with_guide.launch`: includes TurtleBot3 navigation plus guide system.
- Created tests for semantic map, command parser, navigation manager, local search manager, vision detector, and launch files.

## Task 1: Create ROS package skeleton

**Files:**
- Create: `catkin_ws/src/event_guide_robot/package.xml`
- Create: `catkin_ws/src/event_guide_robot/CMakeLists.txt`
- Create directories: `launch/`, `config/`, `scripts/`, `test/`

- [x] Create package with dependencies on `rospy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `move_base_msgs`, `actionlib`, `cv_bridge`.
- [ ] Run `catkin_make` from the workspace root. Not run locally: `catkin_make` is not installed in this environment.
- [ ] Expected result: package configures without missing dependency errors.

## Task 2: Create semantic map config

**Files:**
- Create: `catkin_ws/src/event_guide_robot/config/semantic_map.yaml`

- [x] Add 4 initial zones with measured poses from `puntos.txt`.
- [x] Include `aliases`, `nav_goal`, `search_waypoints`, and `labels` for each zone.
- [x] Add at least two MWC-style stands per zone.
- [x] Keep numeric `marker_id` fields as logical IDs usable by ArUco/AprilTag or a future number detector.
- [x] Validate YAML loading with Python.

## Task 3: Test semantic map shape

**Files:**
- Create: `catkin_ws/src/event_guide_robot/test/test_semantic_map.py`

- [x] Write tests asserting every zone has `aliases`, `nav_goal`, `search_waypoints`, and `labels`.
- [x] Write tests asserting every `nav_goal` has numeric `x`, `y`, and `yaw`.
- [x] Write tests asserting every zone has at least two marker labels.
- [x] Run the test with pytest.
- [x] Expected result: tests pass for the semantic map.

## Task 4: Implement command parser

**Files:**
- Create: `catkin_ws/src/event_guide_robot/scripts/semantic_planner_node.py`
- Create: `catkin_ws/src/event_guide_robot/test/test_command_parser.py`

- [x] Implement normalization: lowercase, trim whitespace, remove repeated spaces.
- [x] Implement alias matching against zone aliases and label aliases.
- [x] Return a structured result containing zone ID, label ID, display name, marker ID, nav goal, and waypoints.
- [x] Add tests for matching a zone by name, matching a label by alias, and returning no match for unknown text.
- [x] Publish `/guide/plan`, `/guide/state`, and `/guide/result`.

## Task 5: Implement navigation manager

**Files:**
- Create: `catkin_ws/src/event_guide_robot/scripts/navigation_manager_node.py`

- [x] Create a `move_base` action client.
- [x] Convert `{x, y, yaw}` to `MoveBaseGoal` in the `map` frame.
- [x] Publish `/guide/state` values: `NAVIGATE_TO_ZONE`, `NAVIGATION_SUCCEEDED`, `NAVIGATION_FAILED`.
- [x] Add timeout handling.
- [x] Add pure helper tests for quaternion conversion, JSON parsing, and goal construction.
- [ ] Verify with a known safe goal in RViz/simulation.

## Task 6: Implement local search manager

**Files:**
- Create: `catkin_ws/src/event_guide_robot/scripts/local_search_manager_node.py`
- Create: `catkin_ws/src/event_guide_robot/config/search_params.yaml`

- [x] Load rotation speed, rotation duration, detection timeout, and control rate from YAML/ROS params.
- [x] Publish slow rotation commands on `/cmd_vel`.
- [x] Subscribe to `/vision/detections`.
- [x] Stop the robot by publishing zero velocity when target is found or timeout expires.
- [x] Require `stable: true` before accepting a detection.
- [ ] Iterate through search waypoints if the first scan does not find the target.

## Task 7: Implement vision MVP

**Files:**
- Create: `catkin_ws/src/event_guide_robot/scripts/vision_detector_node.py`

- [x] Subscribe to camera image topic.
- [x] Convert ROS image to OpenCV image using `cv_bridge` when available.
- [x] Detect ArUco marker IDs when `cv2.aruco` is available.
- [x] Map marker ID to label ID using `semantic_map.yaml`.
- [x] Publish detections with marker ID, label ID, confidence, `stable`, and timestamp.
- [x] Require repeated detections over multiple frames before declaring stable detection.
- [ ] Decide whether final demo uses ArUco/AprilTag, number detector, or hybrid fallback.

## Task 8: Create launch file

**Files:**
- Create: `catkin_ws/src/event_guide_robot/launch/guide_system.launch`

- [x] Launch semantic planner, navigation manager, local search manager, and vision detector.
- [x] Load `semantic_map.yaml` and `search_params.yaml` as ROS params.
- [x] Make camera topic configurable.
- [ ] Run `roslaunch event_guide_robot guide_system.launch`.
- [ ] Expected result: all nodes start and publish/log readiness.

## Local verification completed

```text
python3 -m pytest catkin_ws/src/event_guide_robot/test -q
25 passed

python3 -m py_compile catkin_ws/src/event_guide_robot/scripts/*.py
OK
```

XML and YAML parsing were also checked locally. ROS/catkin validation remains pending because this environment does not provide `catkin_make` or `catkin`.

## Task 9: End-to-end demo validation

**Files:**
- Modify as needed based on real robot/simulation results.

- [ ] Start TurtleBot3 bringup.
- [ ] Start camera.
- [ ] Start navigation stack with the generated map.
- [ ] Start guide system launch.
- [ ] Send a command on `/guide/command`.
- [ ] Confirm the robot reaches the semantic zone.
- [ ] Confirm local search rotates or visits waypoints.
- [ ] Confirm visual detector publishes target found.
- [ ] Confirm robot stops and reports success.

## Self-review

- Spec coverage: README and AGENTS define navigation, semantic map, local search, vision MVP, demo criteria, risks, and validation.
- Placeholder scan: no implementation step depends on undefined future choices except real map coordinates, which must be measured from RViz/hardware.
- Type consistency: node names, config file names, and topic names match across README, AGENTS, and this plan.
