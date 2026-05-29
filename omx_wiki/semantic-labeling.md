# Semantic labeling for the Event Guide Robot

Tags: ros1, turtlebot3, semantic-map, navigation, vision
Category: reference

## Summary

Semantic labels are stored as a separate YAML layer over the ROS navigation map. Do not edit `map.pgm` or overload `map.yaml` with event semantics. Use `semantic_map.yaml` to map human names and aliases to `map` frame poses, search waypoints, and visual labels.

Main reference: [[project-summary]] and `docs/semantic-labeling-guide.md`.

## Core model

- A **zone** is a general area of the event map.
- A **label** is a specific target inside a zone, such as a stand name, sign, marker, or paper label.
- A zone should have one `nav_goal` and several `search_waypoints`.
- A label should have aliases and a numeric visual/logical identifier. The current field is `marker_id`; it can represent an ArUco/AprilTag ID or a visible number if the team chooses a number detector.

## Current semantic map

Current file:

```text
catkin_ws/src/event_guide_robot/config/semantic_map.yaml
```

It contains four measured zones:

- `zona_arriba`
- `zona_izquierda`
- `zona_abajo`
- `zona_derecha`

Each zone currently has two MWC-style stands:

| Zone | Labels |
| --- | --- |
| `zona_arriba` | Qualcomm AI Hub, Samsung Galaxy Experience |
| `zona_izquierda` | Telefonica Open Gateway, Nokia Networks Lab |
| `zona_abajo` | Ericsson 5G Arena, GSMA Innovation City |
| `zona_derecha` | Meta XR Showcase, NVIDIA Edge AI |

The current yaw values are inferred toward the centroid of the measured points and should be checked in RViz before the final demo.

## Recommended YAML shape

```yaml
zones:
  zona_startups:
    name: "Zona Startups"
    aliases: ["startups", "zona startups", "emprendedores"]
    nav_goal:
      x: 2.10
      y: 1.80
      yaw: 1.57
    search_waypoints:
      - {x: 2.10, y: 1.80, yaw: 0.00}
      - {x: 2.50, y: 1.80, yaw: 1.57}
      - {x: 2.10, y: 2.20, yaw: 3.14}
    labels:
      qualcomm:
        aliases: ["qualcomm", "qcom"]
        marker_id: 17
```

## Coordinate capture workflow

1. Launch navigation with the generated map.
2. In RViz, use `2D Nav Goal` to choose safe poses.
3. Echo `/move_base_simple/goal` to copy positions and orientation.
4. Alternatively move the robot physically and echo `/amcl_pose`.
5. Store measured coordinates in `semantic_map.yaml`.

## First implementation target

The first implementation target has been reached in code:

- semantic planner resolves user commands to zone/stand plans;
- navigation manager sends the zone goal to `move_base`;
- local search manager rotates with `/cmd_vel` and waits for stable detections;
- vision detector publishes generic JSON detections and can use ArUco when available.

Runtime validation with ROS/catkin, RViz, `move_base`, camera, `/scan`, and `/cmd_vel` remains pending.
