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
        "safe_navigation_manager_node.py",
        "navigation_manager_node.py",
        "local_search_manager_node.py",
        "vision_detector_node.py",
    }.issubset(node_types)


def test_guide_system_defaults_to_safe_navigation_backend():
    launch_file = PACKAGE_ROOT / "launch" / "guide_system.launch"

    root = ET.parse(launch_file).getroot()
    args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}

    assert args["navigation_backend"] == "safe"
    assert args["safe_navigation_params"] == "$(find event_guide_robot)/config/safe_navigation.yaml"
    assert (PACKAGE_ROOT / "config" / "safe_navigation.yaml").is_file()


def test_navigation_with_guide_launch_includes_navigation_and_guide():
    launch_file = PACKAGE_ROOT / "launch" / "navigation_with_guide.launch"

    root = ET.parse(launch_file).getroot()
    includes = [include.attrib["file"] for include in root.findall("include")]

    assert any("turtlebot3_navigation.launch" in include for include in includes)
    assert any("guide_system.launch" in include for include in includes)


def test_navigation_launch_passes_safe_backend_to_guide_system():
    launch_file = PACKAGE_ROOT / "launch" / "navigation_with_guide.launch"

    root = ET.parse(launch_file).getroot()
    args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
    guide_include = next(
        include
        for include in root.findall("include")
        if "guide_system.launch" in include.attrib["file"]
    )
    passed_args = {
        arg.attrib["name"]: arg.attrib.get("value")
        for arg in guide_include.findall("arg")
    }

    assert args["navigation_backend"] == "safe"
    assert args["safe_navigation_params"] == "$(find event_guide_robot)/config/safe_navigation.yaml"
    assert passed_args["navigation_backend"] == "$(arg navigation_backend)"
    assert passed_args["safe_navigation_params"] == "$(arg safe_navigation_params)"


def test_navigation_launch_relaxes_final_yaw_by_default():
    launch_file = PACKAGE_ROOT / "launch" / "navigation_with_guide.launch"

    root = ET.parse(launch_file).getroot()
    args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
    params = {param.attrib["name"]: param.attrib.get("value") for param in root.findall("param")}

    assert args["yaw_goal_tolerance"] == "6.283185307"
    assert params["/move_base/DWAPlannerROS/yaw_goal_tolerance"] == "$(arg yaw_goal_tolerance)"
    assert params["/move_base/TrajectoryPlannerROS/yaw_goal_tolerance"] == "$(arg yaw_goal_tolerance)"


def test_navigation_launch_loads_move_base_safety_overrides():
    launch_file = PACKAGE_ROOT / "launch" / "navigation_with_guide.launch"

    root = ET.parse(launch_file).getroot()
    args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
    rosparams = [
        rosparam.attrib
        for rosparam in root.findall("rosparam")
        if rosparam.attrib.get("command") == "load"
    ]

    assert args["move_base_safety_params"] == "$(find event_guide_robot)/config/move_base_safety.yaml"
    assert any(
        rosparam.get("file") == "$(arg move_base_safety_params)"
        for rosparam in rosparams
    )
    assert (PACKAGE_ROOT / "config" / "move_base_safety.yaml").is_file()


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
