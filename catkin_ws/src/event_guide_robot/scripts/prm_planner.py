#!/usr/bin/env python3
"""Pure PRM path planner helpers for occupancy-grid maps.

The functions in this module avoid ROS imports so they can be unit-tested on a
regular development machine. The ROS node adapter lives in ``prm_planner_node``.
"""

import heapq
import math
import random
from pathlib import Path


class NoPathError(RuntimeError):
    """Raised when PRM cannot connect start and goal."""


class GridMap:
    """Small occupancy grid wrapper using map-frame world coordinates."""

    def __init__(self, width, height, resolution, origin_x, origin_y, occupied):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.occupied = set(occupied)

    def world_to_cell(self, x, y):
        cell_x = int(math.floor((float(x) - self.origin_x) / self.resolution))
        cell_y = int(math.floor((float(y) - self.origin_y) / self.resolution))
        return cell_x, cell_y

    def cell_to_world(self, cell_x, cell_y):
        return (
            self.origin_x + (int(cell_x) + 0.5) * self.resolution,
            self.origin_y + (int(cell_y) + 0.5) * self.resolution,
        )

    def in_bounds(self, cell_x, cell_y):
        return 0 <= cell_x < self.width and 0 <= cell_y < self.height

    def is_free_cell(self, cell_x, cell_y):
        return self.in_bounds(cell_x, cell_y) and (cell_x, cell_y) not in self.occupied

    def is_free_world(self, x, y):
        return self.is_free_cell(*self.world_to_cell(x, y))

    def free_cell_centers(self):
        centers = []
        for cell_y in range(self.height):
            for cell_x in range(self.width):
                if self.is_free_cell(cell_x, cell_y):
                    x, y = self.cell_to_world(cell_x, cell_y)
                    centers.append({"x": x, "y": y, "yaw": 0.0})
        return centers

    def line_is_free(self, start_x, start_y, goal_x, goal_y):
        if not self.is_free_world(start_x, start_y) or not self.is_free_world(goal_x, goal_y):
            return False

        distance = math.hypot(float(goal_x) - float(start_x), float(goal_y) - float(start_y))
        steps = max(1, int(math.ceil(distance / (self.resolution * 0.5))))
        for index in range(steps + 1):
            ratio = float(index) / float(steps)
            x = float(start_x) + (float(goal_x) - float(start_x)) * ratio
            y = float(start_y) + (float(goal_y) - float(start_y)) * ratio
            if not self.is_free_world(x, y):
                return False
        return True


def _read_token(stream):
    token = bytearray()
    while True:
        char = stream.read(1)
        if not char:
            return bytes(token) if token else None
        if char == b"#":
            stream.readline()
            if token:
                return bytes(token)
            continue
        if char.isspace():
            if token:
                return bytes(token)
            continue
        token.extend(char)


def read_pgm(path):
    """Read a P2 or P5 PGM file and return ``(width, height, pixels)``."""
    with Path(path).open("rb") as stream:
        magic = _read_token(stream)
        if magic not in (b"P2", b"P5"):
            raise ValueError("Unsupported PGM format: {}".format(magic))

        width = int(_read_token(stream))
        height = int(_read_token(stream))
        max_value = int(_read_token(stream))
        if max_value <= 0 or max_value > 255:
            raise ValueError("Unsupported PGM max value: {}".format(max_value))

        if magic == b"P5":
            raw_pixels = stream.read(width * height)
            if len(raw_pixels) != width * height:
                raise ValueError("PGM file ended before all pixels were read")
            pixels = list(raw_pixels)
        else:
            pixels = [int(_read_token(stream)) for _ in range(width * height)]

    return width, height, pixels


def load_occupancy_grid_from_yaml(map_yaml_path, robot_radius_m=0.0):
    """Load a ROS map YAML/PGM pair into a ``GridMap``."""
    import yaml

    map_yaml_path = Path(map_yaml_path)
    with map_yaml_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)

    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path

    width, height, pixels = read_pgm(image_path)
    resolution = float(metadata["resolution"])
    origin = metadata.get("origin", [0.0, 0.0, 0.0])
    occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
    free_threshold = int(round((1.0 - occupied_threshold) * 255.0))

    occupied = set()
    for image_y in range(height):
        for image_x in range(width):
            pixel = pixels[image_y * width + image_x]
            cell_y = height - 1 - image_y
            if pixel <= free_threshold:
                occupied.add((image_x, cell_y))

    if robot_radius_m > 0.0:
        occupied = inflate_occupied_cells(
            occupied, width, height, int(math.ceil(float(robot_radius_m) / resolution))
        )

    return GridMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin[0],
        origin_y=origin[1],
        occupied=occupied,
    )


def inflate_occupied_cells(occupied, width, height, radius_cells):
    """Return occupied cells inflated by a circular robot footprint."""
    if radius_cells <= 0:
        return set(occupied)

    inflated = set(occupied)
    for cell_x, cell_y in occupied:
        for delta_y in range(-radius_cells, radius_cells + 1):
            for delta_x in range(-radius_cells, radius_cells + 1):
                if math.hypot(delta_x, delta_y) > radius_cells:
                    continue
                candidate = (cell_x + delta_x, cell_y + delta_y)
                if 0 <= candidate[0] < width and 0 <= candidate[1] < height:
                    inflated.add(candidate)
    return inflated


def _distance(first, second):
    return math.hypot(float(first["x"]) - float(second["x"]), float(first["y"]) - float(second["y"]))


def _pose_with_yaw(pose, yaw):
    return {"x": float(pose["x"]), "y": float(pose["y"]), "yaw": float(yaw)}


def sample_free_poses(grid, sample_count, random_seed):
    """Return deterministic random free poses from the grid."""
    free_poses = grid.free_cell_centers()
    sample_count = max(0, int(sample_count))
    if len(free_poses) <= sample_count:
        return free_poses

    rng = random.Random(random_seed)
    return rng.sample(free_poses, sample_count)


def build_roadmap(grid, poses, connection_radius):
    """Build an undirected roadmap between visible nearby poses."""
    connection_radius = float(connection_radius)
    graph = {index: [] for index in range(len(poses))}
    for first_index, first in enumerate(poses):
        for second_index in range(first_index + 1, len(poses)):
            second = poses[second_index]
            distance = _distance(first, second)
            if distance > connection_radius:
                continue
            if not grid.line_is_free(first["x"], first["y"], second["x"], second["y"]):
                continue
            graph[first_index].append((second_index, distance))
            graph[second_index].append((first_index, distance))
    return graph


def shortest_path(graph, start_index, goal_index):
    """Return node indexes for the shortest path through ``graph``."""
    frontier = [(0.0, start_index)]
    costs = {start_index: 0.0}
    parents = {start_index: None}

    while frontier:
        cost, node = heapq.heappop(frontier)
        if node == goal_index:
            break
        if cost > costs[node]:
            continue
        for neighbor, edge_cost in graph[node]:
            new_cost = cost + edge_cost
            if neighbor not in costs or new_cost < costs[neighbor]:
                costs[neighbor] = new_cost
                parents[neighbor] = node
                heapq.heappush(frontier, (new_cost, neighbor))

    if goal_index not in parents:
        raise NoPathError("PRM could not connect start and goal")

    path = []
    node = goal_index
    while node is not None:
        path.append(node)
        node = parents[node]
    path.reverse()
    return path


def plan_prm_path(
    grid,
    start,
    goal,
    sample_count=250,
    connection_radius=0.8,
    random_seed=13,
):
    """Plan a map-frame path from ``start`` to ``goal`` using PRM."""
    start_pose = _pose_with_yaw(start, start.get("yaw", 0.0))
    goal_pose = _pose_with_yaw(goal, goal.get("yaw", 0.0))

    if not grid.is_free_world(start_pose["x"], start_pose["y"]):
        raise NoPathError("PRM start pose is occupied or outside the map")
    if not grid.is_free_world(goal_pose["x"], goal_pose["y"]):
        raise NoPathError("PRM goal pose is occupied or outside the map")

    samples = sample_free_poses(grid, sample_count, random_seed)
    poses = [start_pose] + samples + [goal_pose]
    graph = build_roadmap(grid, poses, connection_radius)
    indexes = shortest_path(graph, 0, len(poses) - 1)
    path = [poses[index] for index in indexes]
    path[0] = start_pose
    path[-1] = goal_pose
    return path
