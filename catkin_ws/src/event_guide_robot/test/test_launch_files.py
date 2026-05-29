#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_guide_system_launch_is_valid_xml_and_starts_core_nodes():
    launch_file = PACKAGE_ROOT / "launch" / "guide_system.launch"

    root = ET.parse(launch_file).getroot()
    node_types = {node.attrib["type"] for node in root.findall("node")}

    assert root.tag == "launch"
    assert {
        "semantic_planner_node.py",
        "navigation_manager_node.py",
        "local_search_manager_node.py",
        "vision_detector_node.py",
    }.issubset(node_types)


def test_navigation_with_guide_launch_includes_navigation_and_guide():
    launch_file = PACKAGE_ROOT / "launch" / "navigation_with_guide.launch"

    root = ET.parse(launch_file).getroot()
    includes = [include.attrib["file"] for include in root.findall("include")]

    assert any("turtlebot3_navigation.launch" in include for include in includes)
    assert any("guide_system.launch" in include for include in includes)


def test_navigation_launch_relaxes_final_yaw_by_default():
    launch_file = PACKAGE_ROOT / "launch" / "navigation_with_guide.launch"

    root = ET.parse(launch_file).getroot()
    args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
    params = {param.attrib["name"]: param.attrib.get("value") for param in root.findall("param")}

    assert args["yaw_goal_tolerance"] == "6.283185307"
    assert params["/move_base/DWAPlannerROS/yaw_goal_tolerance"] == "$(arg yaw_goal_tolerance)"
    assert params["/move_base/TrajectoryPlannerROS/yaw_goal_tolerance"] == "$(arg yaw_goal_tolerance)"


def test_navigation_launch_uses_packaged_map_by_default():
    launch_file = PACKAGE_ROOT / "launch" / "navigation_with_guide.launch"

    root = ET.parse(launch_file).getroot()
    args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}

    assert args["map_file"] == "$(find event_guide_robot)/maps/map.yaml"
    assert (PACKAGE_ROOT / "maps" / "map.yaml").is_file()
    assert (PACKAGE_ROOT / "maps" / "map.pgm").is_file()
