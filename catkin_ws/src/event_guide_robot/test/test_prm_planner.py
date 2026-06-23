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
    assert any(
        first["x"] < 3.0 < second["x"] or second["x"] < 3.0 < first["x"]
        for first, second in zip(path, path[1:])
    )
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


def test_grid_map_reports_clearance_from_nearest_obstacle():
    prm = load_prm_module()
    grid = prm.GridMap(
        width=5,
        height=3,
        resolution=0.5,
        origin_x=0.0,
        origin_y=0.0,
        occupied={(2, 1)},
    )

    assert grid.clearance_world(1.25, 0.75) == pytest.approx(0.0)
    assert grid.clearance_world(2.25, 0.75) == pytest.approx(1.0)
    assert grid.clearance_world(-0.25, 0.75) == pytest.approx(0.0)


def test_line_clearance_returns_tightest_sampled_clearance():
    prm = load_prm_module()
    grid = prm.GridMap(
        width=5,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        occupied={(2, 2)},
    )

    assert grid.line_clearance(0.5, 2.5, 4.5, 2.5) == pytest.approx(0.0)
    assert grid.line_clearance(0.5, 4.5, 4.5, 4.5) == pytest.approx(2.0)


def test_roadmap_skips_edges_below_min_clearance():
    prm = load_prm_module()
    grid = prm.GridMap(
        width=5,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        occupied={(2, 2)},
    )
    poses = [
        {"x": 0.5, "y": 2.5, "yaw": 0.0},
        {"x": 4.5, "y": 2.5, "yaw": 0.0},
        {"x": 0.5, "y": 4.5, "yaw": 0.0},
    ]

    graph = prm.build_roadmap(grid, poses, connection_radius=5.0, min_clearance_m=1.0)

    assert graph[0] == [(2, pytest.approx(2.0))]
    assert all(neighbor != 1 for neighbor, _ in graph[0])


def test_clearance_weight_prefers_wider_path_over_shorter_edge():
    prm = load_prm_module()
    grid = prm.GridMap(
        width=7,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        occupied={(3, 1)},
    )
    poses = [
        {"x": 0.5, "y": 2.5, "yaw": 0.0},
        {"x": 6.5, "y": 2.5, "yaw": 0.0},
        {"x": 3.5, "y": 4.5, "yaw": 0.0},
    ]

    unweighted = prm.build_roadmap(grid, poses, connection_radius=7.0, clearance_weight=0.0)
    weighted = prm.build_roadmap(grid, poses, connection_radius=7.0, clearance_weight=4.0)

    assert prm.shortest_path(unweighted, 0, 1) == [0, 1]
    assert prm.shortest_path(weighted, 0, 1) == [0, 2, 1]
