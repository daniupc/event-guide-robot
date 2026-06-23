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
        "prm_planner_node.py",
        "prm_trajectory_follower_node.py",
        "local_search_manager_node.py",
        "vision_detector_node.py",
    }.issubset(node_types)


def test_guide_system_launch_routes_navigation_through_prm_by_default():
    launch_file = PACKAGE_ROOT / "launch" / "guide_system.launch"

    root = ET.parse(launch_file).getroot()
    args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
    nodes = {node.attrib["name"]: node for node in root.findall("node")}

    follower_params = {
        param.attrib["name"]: param.attrib.get("value")
        for param in nodes["prm_trajectory_follower_node"].findall("param")
    }
    prm_params = {
        param.attrib["name"]: param.attrib.get("value")
        for param in nodes["prm_planner_node"].findall("param")
    }

    assert args["navigation_plan_topic"] == "/guide/navigation_plan"
    assert follower_params["plan_topic"] == "$(arg navigation_plan_topic)"
    assert prm_params["input_plan_topic"] == "$(arg semantic_plan_topic)"
    assert prm_params["output_plan_topic"] == "$(arg navigation_plan_topic)"
    assert prm_params["map_file"] == "$(arg map_file)"


def test_guide_system_launch_uses_prm_follower_instead_of_move_base_manager_by_default():
    launch_file = PACKAGE_ROOT / "launch" / "guide_system.launch"

    root = ET.parse(launch_file).getroot()
    node_types = {node.attrib["type"] for node in root.findall("node")}

    assert "prm_trajectory_follower_node.py" in node_types
    assert "navigation_manager_node.py" not in node_types


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


def test_launch_files_default_to_turtlebot3_rpicamera_topic():
    expected_topic = "/raspicam_node/image"
    for launch_name in ("guide_system.launch", "navigation_with_guide.launch"):
        root = ET.parse(PACKAGE_ROOT / "launch" / launch_name).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}

        assert args["image_topic"] == expected_topic
