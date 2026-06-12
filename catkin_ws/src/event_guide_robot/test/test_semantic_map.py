#!/usr/bin/env python3
import math
from pathlib import Path

import yaml


SEMANTIC_MAP = Path(__file__).resolve().parents[1] / "config" / "semantic_map.yaml"


def load_map():
    with SEMANTIC_MAP.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_semantic_map_has_required_top_level_sections():
    data = load_map()

    assert data["metadata"]["frame_id"] == "map"
    assert isinstance(data["zones"], dict)
    assert data["zones"]


def test_every_zone_has_required_navigation_shape():
    data = load_map()

    for zone_id, zone in data["zones"].items():
        assert zone["name"], zone_id
        assert zone["aliases"], zone_id
        assert isinstance(zone["labels"], dict), zone_id

        goal = zone["nav_goal"]
        for key in ("x", "y", "yaw"):
            assert isinstance(goal[key], (int, float)), f"{zone_id}.nav_goal.{key}"
            assert math.isfinite(goal[key]), f"{zone_id}.nav_goal.{key}"

        assert zone["search_waypoints"], zone_id
        for index, waypoint in enumerate(zone["search_waypoints"]):
            for key in ("x", "y", "yaw"):
                assert isinstance(waypoint[key], (int, float)), (
                    f"{zone_id}.search_waypoints[{index}].{key}"
                )
                assert math.isfinite(waypoint[key]), (
                    f"{zone_id}.search_waypoints[{index}].{key}"
                )


def test_initial_measured_zones_are_present():
    data = load_map()

    assert set(data["zones"]) == {
        "zona_arriba",
        "zona_izquierda",
        "zona_abajo",
        "zona_derecha",
    }


def test_pasillo_map_and_measured_goals_are_configured():
    data = load_map()

    assert data["metadata"]["map_file"] == "../maps/mapa_passadis.yaml"

    expected_goals = {
        "zona_arriba": (-1.4704312086105347, -4.636334419250488),
        "zona_izquierda": (-1.972220540046692, -0.48459815979003906),
        "zona_abajo": (0.3927302062511444, -1.5970544815063477),
        "zona_derecha": (0.6981806755065918, -4.179182052612305),
    }
    for zone_id, (expected_x, expected_y) in expected_goals.items():
        goal = data["zones"][zone_id]["nav_goal"]

        assert goal["x"] == expected_x
        assert goal["y"] == expected_y
        assert goal["yaw"] == 0.0
        assert data["zones"][zone_id]["search_waypoints"] == [goal]


def test_every_zone_has_at_least_two_marker_labels():
    data = load_map()

    for zone_id, zone in data["zones"].items():
        assert len(zone["labels"]) >= 2, zone_id
        for label_id, label in zone["labels"].items():
            assert label["aliases"], f"{zone_id}.{label_id}"
            assert isinstance(label["marker_id"], int), f"{zone_id}.{label_id}.marker_id"
            assert label["marker_id"] > 0, f"{zone_id}.{label_id}.marker_id"
