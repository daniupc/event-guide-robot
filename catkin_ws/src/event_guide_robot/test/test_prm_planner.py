#!/usr/bin/env python3
import importlib.util
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PRM_PLANNER_PATH = PACKAGE_ROOT / "scripts" / "prm_planner.py"


def load_prm_module():
    spec = importlib.util.spec_from_file_location("prm_planner", PRM_PLANNER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_grid_map_converts_between_world_and_grid_coordinates():
    prm = load_prm_module()
    grid = prm.GridMap(
        width=4,
        height=3,
        resolution=0.5,
        origin_x=-1.0,
        origin_y=2.0,
        occupied={(2, 1)},
    )

    assert grid.world_to_cell(-0.75, 2.25) == (0, 0)
    assert grid.cell_to_world(2, 1) == pytest.approx((0.25, 2.75))
    assert grid.is_free_cell(1, 1) is True
    assert grid.is_free_cell(2, 1) is False


def test_prm_finds_path_through_gap_in_wall():
    prm = load_prm_module()
    occupied = {(3, y) for y in range(6) if y != 2}
    grid = prm.GridMap(
        width=7,
        height=6,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        occupied=occupied,
    )

    path = prm.plan_prm_path(
        grid,
        start={"x": 1.5, "y": 2.5, "yaw": 0.0},
        goal={"x": 5.5, "y": 2.5, "yaw": 1.0},
        sample_count=80,
        connection_radius=3.0,
        random_seed=7,
    )

    assert path[0] == {"x": 1.5, "y": 2.5, "yaw": 0.0}
    assert path[-1] == {"x": 5.5, "y": 2.5, "yaw": 1.0}
    assert any(3.0 <= pose["x"] < 4.0 and 2.0 <= pose["y"] < 3.0 for pose in path)
    for first, second in zip(path, path[1:]):
        assert grid.line_is_free(first["x"], first["y"], second["x"], second["y"])


def test_prm_raises_when_goal_is_blocked_from_start():
    prm = load_prm_module()
    occupied = {(2, y) for y in range(5)}
    grid = prm.GridMap(
        width=5,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        occupied=occupied,
    )

    with pytest.raises(prm.NoPathError):
        prm.plan_prm_path(
            grid,
            start={"x": 0.5, "y": 2.5, "yaw": 0.0},
            goal={"x": 4.5, "y": 2.5, "yaw": 0.0},
            sample_count=40,
            connection_radius=2.0,
            random_seed=3,
        )
