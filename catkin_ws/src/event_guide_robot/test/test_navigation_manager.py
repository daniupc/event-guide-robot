#!/usr/bin/env python3
import importlib.util
import json
import math
import pytest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAVIGATION_MANAGER_PATH = PACKAGE_ROOT / "scripts" / "navigation_manager_node.py"


def load_navigation_manager_module():
    spec = importlib.util.spec_from_file_location(
        "navigation_manager_node", NAVIGATION_MANAGER_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakePoint:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None


class FakeQuaternion:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.w = None


class FakePose:
    def __init__(self):
        self.position = FakePoint()
        self.orientation = FakeQuaternion()


class FakeHeader:
    def __init__(self):
        self.frame_id = None
        self.stamp = None


class FakePoseStamped:
    def __init__(self):
        self.header = FakeHeader()
        self.pose = FakePose()


class FakeMoveBaseGoal:
    def __init__(self):
        self.target_pose = FakePoseStamped()


class FakeMoveBaseMsgClasses:
    MoveBaseGoal = FakeMoveBaseGoal


def test_yaw_to_quaternion_for_zero_yaw():
    navigation_manager = load_navigation_manager_module()

    quaternion = navigation_manager.yaw_to_quaternion(0.0)

    assert quaternion == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


def test_yaw_to_quaternion_for_half_pi_yaw():
    navigation_manager = load_navigation_manager_module()

    quaternion = navigation_manager.yaw_to_quaternion(math.pi / 2.0)

    assert quaternion["x"] == 0.0
    assert quaternion["y"] == 0.0
    assert quaternion["z"] == pytest.approx(math.sqrt(0.5))
    assert quaternion["w"] == pytest.approx(math.sqrt(0.5))


def test_parse_plan_json_returns_plan_dictionary():
    navigation_manager = load_navigation_manager_module()
    plan = {
        "status": "FOUND",
        "zone_id": "zona_arriba",
        "nav_goal": {"x": 1.25, "y": -0.5, "yaw": 1.0},
        "search_waypoints": [],
    }

    parsed = navigation_manager.parse_plan_json(json.dumps(plan))

    assert parsed == plan


def test_build_move_base_goal_uses_map_frame_and_pose_from_plan():
    navigation_manager = load_navigation_manager_module()
    plan = {
        "status": "FOUND",
        "nav_goal": {"x": 1.25, "y": -0.5, "yaw": math.pi / 2.0},
    }

    goal = navigation_manager.build_move_base_goal(
        plan, move_base_msg_classes=FakeMoveBaseMsgClasses, stamp="test-stamp"
    )

    assert isinstance(goal, FakeMoveBaseGoal)
    assert goal.target_pose.header.frame_id == "map"
    assert goal.target_pose.header.stamp == "test-stamp"
    assert goal.target_pose.pose.position.x == 1.25
    assert goal.target_pose.pose.position.y == -0.5
    assert goal.target_pose.pose.position.z == 0.0
    assert goal.target_pose.pose.orientation.x == 0.0
    assert goal.target_pose.pose.orientation.y == 0.0
    assert goal.target_pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert goal.target_pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))
