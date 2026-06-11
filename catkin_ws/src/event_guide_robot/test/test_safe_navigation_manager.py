import importlib.util
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "safe_navigation_manager_node.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("safe_navigation_manager_node", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_grid(module, width, height, occupied_cells, resolution=0.1):
    data = [0] * (width * height)
    for x, y in occupied_cells:
        data[y * width + x] = 100
    return module.GridMap(width, height, resolution, 0.0, 0.0, data)


def test_planner_routes_around_inflated_obstacle():
    nav = load_module()
    occupied = {(7, y) for y in range(16) if y not in (10, 11, 12)}
    grid = make_grid(nav, 16, 16, occupied)

    navigation_grid = nav.build_navigation_grid(
        grid,
        min_clearance_m=0.11,
        preferred_clearance_m=0.30,
    )
    path = nav.plan_clearance_path(navigation_grid, (0.25, 0.25), (1.35, 1.35))

    assert path, "expected a path through the free gap"
    assert len(path) >= 2
    assert all(navigation_grid.is_world_safe(point) for point in path)
    assert any(x > 0.75 and y > 1.05 for x, y in path)


def test_goal_close_to_wall_snaps_to_nearest_safe_cell():
    nav = load_module()
    occupied = {(0, y) for y in range(8)}
    grid = make_grid(nav, 10, 8, occupied)

    navigation_grid = nav.build_navigation_grid(
        grid,
        min_clearance_m=0.20,
        preferred_clearance_m=0.35,
    )
    snapped = nav.find_nearest_safe_world(navigation_grid, (0.12, 0.35), 0.8)

    assert snapped is not None
    assert snapped[0] >= 0.25
    assert navigation_grid.is_world_safe(snapped)


def test_sensor_freshness_gate_rejects_old_data():
    nav = load_module()

    assert nav.sensor_is_fresh(10.0, 10.2, 0.5)
    assert not nav.sensor_is_fresh(10.0, 10.6, 0.5)
    assert not nav.sensor_is_fresh(None, 10.0, 0.5)
