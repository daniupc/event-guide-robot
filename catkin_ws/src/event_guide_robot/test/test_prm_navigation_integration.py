#!/usr/bin/env python3
import importlib.util
import json
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


def test_navigation_poses_prefers_prm_navigation_path_when_present():
    navigation_manager = load_navigation_manager_module()
    plan = {
        "status": "FOUND",
        "nav_goal": {"x": 9.0, "y": 9.0, "yaw": 0.0},
        "navigation_path": [
            {"x": 1.0, "y": 2.0, "yaw": 0.0},
            {"x": 3.0, "y": 4.0, "yaw": 1.57},
        ],
    }

    assert navigation_manager.navigation_poses_from_plan(plan) == plan["navigation_path"]


def test_navigation_poses_falls_back_to_nav_goal_for_legacy_plan():
    navigation_manager = load_navigation_manager_module()
    plan = {
        "status": "FOUND",
        "nav_goal": {"x": 9.0, "y": 9.0, "yaw": 0.0},
    }

    assert navigation_manager.navigation_poses_from_plan(plan) == [plan["nav_goal"]]


def test_parse_plan_json_accepts_prm_enriched_plan():
    navigation_manager = load_navigation_manager_module()
    plan = {
        "status": "FOUND",
        "planner": "prm",
        "nav_goal": {"x": 9.0, "y": 9.0, "yaw": 0.0},
        "navigation_path": [{"x": 1.0, "y": 2.0, "yaw": 0.0}],
    }

    assert navigation_manager.parse_plan_json(json.dumps(plan)) == plan
