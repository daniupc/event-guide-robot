#!/usr/bin/env python3
import math
from pathlib import Path

import yaml


SEMANTIC_MAP = Path(__file__).resolve().parents[1] / "config" / "semantic_map.yaml"
MIN_NAV_GOAL_CLEARANCE_M = 0.40


def load_map():
    with SEMANTIC_MAP.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_pgm(path):
    data = path.read_bytes()
    index = 0

    def next_token():
        nonlocal index
        while index < len(data):
            byte = data[index]
            if byte == ord("#"):
                while index < len(data) and data[index] not in (10, 13):
                    index += 1
            elif chr(byte).isspace():
                index += 1
            else:
                break

        start = index
        while index < len(data) and not chr(data[index]).isspace():
            index += 1
        return data[start:index].decode("ascii")

    magic = next_token()
    width = int(next_token())
    height = int(next_token())
    max_value = int(next_token())
    while index < len(data) and chr(data[index]).isspace():
        index += 1

    assert magic == "P5"
    assert max_value == 255
    pixels = data[index : index + width * height]
    assert len(pixels) == width * height
    return width, height, pixels


def load_map_metadata():
    map_yaml = (SEMANTIC_MAP.parent / load_map()["metadata"]["map_file"]).resolve()
    with map_yaml.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    image_path = (map_yaml.parent / metadata["image"]).resolve()
    if not image_path.exists():
        image_path = map_yaml.parent / Path(metadata["image"]).name
    return metadata, image_path


def world_to_pixel(x, y, width, height, resolution, origin):
    pixel_x = (x - origin[0]) / resolution
    pixel_y = height - 1 - ((y - origin[1]) / resolution)
    return pixel_x, pixel_y


def clearance_to_wall_m(goal, metadata, image):
    width, height, pixels = image
    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    pixel_x, pixel_y = world_to_pixel(
        goal["x"], goal["y"], width, height, resolution, origin
    )

    occupied_pixels = [
        (index % width, index // width)
        for index, value in enumerate(pixels)
        if value < 100
    ]
    return min(
        math.hypot(pixel_x - occupied_x, pixel_y - occupied_y) * resolution
        for occupied_x, occupied_y in occupied_pixels
    )


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


def test_every_zone_has_at_least_two_marker_labels():
    data = load_map()

    for zone_id, zone in data["zones"].items():
        assert len(zone["labels"]) >= 2, zone_id
        for label_id, label in zone["labels"].items():
            assert label["aliases"], f"{zone_id}.{label_id}"
            assert isinstance(label["marker_id"], int), f"{zone_id}.{label_id}.marker_id"
            assert label["marker_id"] > 0, f"{zone_id}.{label_id}.marker_id"


def test_navigation_goals_keep_safe_clearance_from_walls():
    data = load_map()
    map_metadata, image_path = load_map_metadata()
    image = load_pgm(image_path)

    for zone_id, zone in data["zones"].items():
        clearance = clearance_to_wall_m(zone["nav_goal"], map_metadata, image)
        assert clearance >= MIN_NAV_GOAL_CLEARANCE_M, (
            f"{zone_id}.nav_goal has only {clearance:.2f}m wall clearance"
        )

