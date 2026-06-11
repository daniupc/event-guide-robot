#!/usr/bin/env python3
"""Clearance-aware guide navigation without using move_base action goals.

The pure planning helpers are intentionally ROS-free so they can be tested on a
normal laptop. The ROS adapter consumes the same /guide/plan messages as the
legacy move_base bridge, plans on /map, and drives /cmd_vel only when pose,
laser and map data are fresh enough.
"""

import heapq
import json
import math


STATE_NAVIGATING = "NAVIGATE_TO_ZONE"
STATE_WAITING_FOR_MAP = "NAVIGATION_WAITING_FOR_MAP"
STATE_WAITING_FOR_POSE = "NAVIGATION_WAITING_FOR_POSE"
STATE_WAITING_FOR_SENSOR = "NAVIGATION_WAITING_FOR_SENSOR"
STATE_BLOCKED = "NAVIGATION_BLOCKED"
STATE_SUCCEEDED = "NAVIGATION_SUCCEEDED"
STATE_FAILED = "NAVIGATION_FAILED"


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def sensor_is_fresh(stamp_sec, now_sec, max_age_sec):
    if stamp_sec is None:
        return False
    return 0.0 <= (now_sec - stamp_sec) <= max_age_sec


class GridMap:
    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.data = list(data)

    def in_bounds(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def index(self, cell):
        x, y = cell
        return y * self.width + x

    def cell_center(self, cell):
        x, y = cell
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )

    def world_to_cell(self, point):
        x = int(math.floor((point[0] - self.origin_x) / self.resolution))
        y = int(math.floor((point[1] - self.origin_y) / self.resolution))
        return (x, y)

    def is_occupied(self, cell, occupied_threshold=50, unknown_is_obstacle=True):
        if not self.in_bounds(cell):
            return True
        value = self.data[self.index(cell)]
        if value < 0:
            return bool(unknown_is_obstacle)
        return value >= occupied_threshold


class NavigationGrid:
    def __init__(self, grid_map, clearance, min_clearance_m, preferred_clearance_m, clearance_penalty):
        self.grid_map = grid_map
        self.clearance = clearance
        self.min_clearance_m = float(min_clearance_m)
        self.preferred_clearance_m = float(preferred_clearance_m)
        self.clearance_penalty = float(clearance_penalty)

    def clearance_m(self, cell):
        if not self.grid_map.in_bounds(cell):
            return 0.0
        value = self.clearance[self.grid_map.index(cell)]
        return value

    def is_cell_safe(self, cell):
        return self.grid_map.in_bounds(cell) and self.clearance_m(cell) >= self.min_clearance_m

    def is_world_safe(self, point):
        return self.is_cell_safe(self.grid_map.world_to_cell(point))

    def traversal_cost(self, cell):
        clearance = self.clearance_m(cell)
        if clearance >= self.preferred_clearance_m:
            return 0.0
        missing = (self.preferred_clearance_m - clearance) / max(self.preferred_clearance_m, 1e-6)
        return self.clearance_penalty * missing


def build_navigation_grid(
    grid_map,
    occupied_threshold=50,
    unknown_is_obstacle=True,
    min_clearance_m=0.28,
    preferred_clearance_m=0.55,
    clearance_penalty=4.0,
):
    total = grid_map.width * grid_map.height
    clearance = [float("inf")] * total
    queue = []

    for y in range(grid_map.height):
        for x in range(grid_map.width):
            cell = (x, y)
            if grid_map.is_occupied(cell, occupied_threshold, unknown_is_obstacle):
                idx = grid_map.index(cell)
                clearance[idx] = 0.0
                heapq.heappush(queue, (0.0, cell))

    neighbors = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]

    while queue:
        distance, cell = heapq.heappop(queue)
        if distance > clearance[grid_map.index(cell)]:
            continue
        for dx, dy, step in neighbors:
            nxt = (cell[0] + dx, cell[1] + dy)
            if not grid_map.in_bounds(nxt):
                continue
            next_distance = distance + step * grid_map.resolution
            idx = grid_map.index(nxt)
            if next_distance < clearance[idx]:
                clearance[idx] = next_distance
                heapq.heappush(queue, (next_distance, nxt))

    return NavigationGrid(
        grid_map,
        clearance,
        min_clearance_m,
        preferred_clearance_m,
        clearance_penalty,
    )


def find_nearest_safe_cell(navigation_grid, seed_cell, max_radius_m):
    grid_map = navigation_grid.grid_map
    if navigation_grid.is_cell_safe(seed_cell):
        return seed_cell

    max_cells = int(math.ceil(max_radius_m / grid_map.resolution))
    queue = [(0.0, seed_cell)]
    visited = {seed_cell}

    while queue:
        _, cell = heapq.heappop(queue)
        dx0 = cell[0] - seed_cell[0]
        dy0 = cell[1] - seed_cell[1]
        if max(abs(dx0), abs(dy0)) > max_cells:
            continue
        if navigation_grid.is_cell_safe(cell):
            return cell
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            nxt = (cell[0] + dx, cell[1] + dy)
            if nxt in visited:
                continue
            visited.add(nxt)
            distance = math.hypot(nxt[0] - seed_cell[0], nxt[1] - seed_cell[1])
            heapq.heappush(queue, (distance, nxt))
    return None


def find_nearest_safe_world(navigation_grid, point, max_radius_m):
    cell = find_nearest_safe_cell(
        navigation_grid,
        navigation_grid.grid_map.world_to_cell(point),
        max_radius_m,
    )
    return None if cell is None else navigation_grid.grid_map.cell_center(cell)


def _heuristic(a, b, resolution):
    return math.hypot(a[0] - b[0], a[1] - b[1]) * resolution


def _reconstruct_path(came_from, current):
    cells = [current]
    while current in came_from:
        current = came_from[current]
        cells.append(current)
    cells.reverse()
    return cells


def line_is_safe(navigation_grid, start, end):
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        if not navigation_grid.is_cell_safe((x0, y0)):
            return False
        if x0 == x1 and y0 == y1:
            return True
        err2 = 2 * err
        if err2 > -dy:
            err -= dy
            x0 += sx
        if err2 < dx:
            err += dx
            y0 += sy


def simplify_cells(navigation_grid, cells):
    if len(cells) <= 2:
        return cells

    simplified = [cells[0]]
    anchor = 0
    probe = 2
    while probe < len(cells):
        if not line_is_safe(navigation_grid, cells[anchor], cells[probe]):
            simplified.append(cells[probe - 1])
            anchor = probe - 1
        probe += 1
    simplified.append(cells[-1])
    return simplified


def plan_clearance_path(navigation_grid, start_world, goal_world, goal_snap_radius_m=0.8):
    grid_map = navigation_grid.grid_map
    start = find_nearest_safe_cell(navigation_grid, grid_map.world_to_cell(start_world), goal_snap_radius_m)
    goal = find_nearest_safe_cell(navigation_grid, grid_map.world_to_cell(goal_world), goal_snap_radius_m)
    if start is None or goal is None:
        return []

    neighbors = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]
    open_set = [(0.0, start)]
    came_from = {}
    cost_so_far = {start: 0.0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            cells = simplify_cells(navigation_grid, _reconstruct_path(came_from, current))
            return [grid_map.cell_center(cell) for cell in cells]

        for dx, dy, step in neighbors:
            nxt = (current[0] + dx, current[1] + dy)
            if not navigation_grid.is_cell_safe(nxt):
                continue
            if dx != 0 and dy != 0:
                if not navigation_grid.is_cell_safe((current[0] + dx, current[1])):
                    continue
                if not navigation_grid.is_cell_safe((current[0], current[1] + dy)):
                    continue
            new_cost = (
                cost_so_far[current]
                + step * grid_map.resolution
                + navigation_grid.traversal_cost(nxt) * grid_map.resolution
            )
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + _heuristic(nxt, goal, grid_map.resolution)
                heapq.heappush(open_set, (priority, nxt))
                came_from[nxt] = current

    return []


def parse_plan_json(text):
    plan = json.loads(text)
    if not isinstance(plan, dict):
        raise ValueError("Guide plan JSON must contain an object")
    if plan.get("status") != "FOUND":
        raise ValueError("Guide plan status is not FOUND")
    nav_goal = plan.get("nav_goal")
    if not isinstance(nav_goal, dict):
        raise ValueError("Guide plan is missing nav_goal")
    return plan


def front_min_range(scan_ranges, angle_min, angle_increment, front_sector_rad):
    best = None
    half_sector = front_sector_rad / 2.0
    for idx, value in enumerate(scan_ranges):
        if value is None or math.isnan(value) or math.isinf(value) or value <= 0.0:
            continue
        angle = angle_min + idx * angle_increment
        if abs(angle) <= half_sector and (best is None or value < best):
            best = value
    return best


def path_to_lookahead(path, robot_xy, lookahead_m):
    if not path:
        return None
    nearest_idx = min(
        range(len(path)),
        key=lambda idx: math.hypot(path[idx][0] - robot_xy[0], path[idx][1] - robot_xy[1]),
    )
    for point in path[nearest_idx:]:
        if math.hypot(point[0] - robot_xy[0], point[1] - robot_xy[1]) >= lookahead_m:
            return point
    return path[-1]


class SafeNavigationManagerNode:
    def __init__(self, rospy_module, messages, tf_module):
        self.rospy = rospy_module
        self.messages = messages
        self.tf = tf_module

        self.base_frame = self.rospy.get_param("~base_frame", "base_footprint")
        self.global_frame = self.rospy.get_param("~global_frame", "map")
        self.navigation_timeout_sec = float(self.rospy.get_param("~navigation_timeout_sec", 90.0))
        self.occupied_threshold = int(self.rospy.get_param("~occupied_threshold", 50))
        self.unknown_is_obstacle = bool(self.rospy.get_param("~unknown_is_obstacle", True))
        self.min_clearance_m = float(self.rospy.get_param("~min_clearance_m", 0.28))
        self.preferred_clearance_m = float(self.rospy.get_param("~preferred_clearance_m", 0.55))
        self.clearance_penalty = float(self.rospy.get_param("~clearance_penalty", 4.0))
        self.goal_snap_radius_m = float(self.rospy.get_param("~goal_snap_radius_m", 0.8))
        self.replan_interval_sec = float(self.rospy.get_param("~replan_interval_sec", 2.0))
        self.control_rate_hz = float(self.rospy.get_param("~control_rate_hz", 10.0))
        self.lookahead_m = float(self.rospy.get_param("~lookahead_m", 0.35))
        self.goal_tolerance_m = float(self.rospy.get_param("~goal_tolerance_m", 0.16))
        self.max_linear_speed = float(self.rospy.get_param("~max_linear_speed_m_s", 0.12))
        self.max_angular_speed = float(self.rospy.get_param("~max_angular_speed_rad_s", 0.50))
        self.heading_gain = float(self.rospy.get_param("~heading_gain", 1.4))
        self.heading_tolerance = float(self.rospy.get_param("~heading_tolerance_rad", 0.45))
        self.scan_stale_sec = float(self.rospy.get_param("~scan_stale_sec", 0.6))
        self.emergency_stop_m = float(self.rospy.get_param("~emergency_stop_m", 0.22))
        self.slowdown_front_m = float(self.rospy.get_param("~slowdown_front_m", 0.45))
        front_sector_deg = float(self.rospy.get_param("~front_sector_deg", 55.0))
        self.front_sector_rad = math.radians(front_sector_deg)

        self.grid_map = None
        self.navigation_grid = None
        self.latest_scan = None
        self.active_plan = None
        self.active_goal = None
        self.display_name = None
        self.started_at = None
        self.path = []
        self.last_plan_time = 0.0
        self.last_state = None

        self.tf_listener = self.tf.TransformListener()
        self.cmd_pub = self.rospy.Publisher("/cmd_vel", messages.Twist, queue_size=10)
        self.state_pub = self.rospy.Publisher("/guide/state", messages.String, queue_size=10)
        self.result_pub = self.rospy.Publisher("/guide/result", messages.String, queue_size=10)
        self.path_pub = self.rospy.Publisher("/guide/debug_path", messages.Path, queue_size=1, latch=True)
        self.plan_sub = self.rospy.Subscriber("/guide/plan", messages.String, self.on_plan)
        self.map_sub = self.rospy.Subscriber("/map", messages.OccupancyGrid, self.on_map)
        self.scan_sub = self.rospy.Subscriber("/scan", messages.LaserScan, self.on_scan)

        period = 1.0 / max(self.control_rate_hz, 0.1)
        self.timer = self.rospy.Timer(self.rospy.Duration(period), self.on_timer)
        self.rospy.on_shutdown(self.stop_robot)

    def publish_state(self, state):
        if state != self.last_state:
            self.state_pub.publish(self.messages.String(data=state))
            self.last_state = state

    def publish_result(self, text):
        self.result_pub.publish(self.messages.String(data=text))

    def stop_robot(self):
        twist = self.messages.Twist()
        self.cmd_pub.publish(twist)

    def on_map(self, message):
        origin = message.info.origin.position
        self.grid_map = GridMap(
            message.info.width,
            message.info.height,
            message.info.resolution,
            origin.x,
            origin.y,
            message.data,
        )
        self.navigation_grid = build_navigation_grid(
            self.grid_map,
            occupied_threshold=self.occupied_threshold,
            unknown_is_obstacle=self.unknown_is_obstacle,
            min_clearance_m=self.min_clearance_m,
            preferred_clearance_m=self.preferred_clearance_m,
            clearance_penalty=self.clearance_penalty,
        )
        self.path = []
        self.last_plan_time = 0.0
        self.rospy.loginfo(
            "Safe navigation map loaded: %sx%s res=%.3f min_clearance=%.2fm preferred=%.2fm",
            self.grid_map.width,
            self.grid_map.height,
            self.grid_map.resolution,
            self.min_clearance_m,
            self.preferred_clearance_m,
        )

    def on_scan(self, message):
        self.latest_scan = message

    def on_plan(self, message):
        try:
            plan = parse_plan_json(message.data)
            goal = plan["nav_goal"]
            self.active_goal = (float(goal["x"]), float(goal["y"]))
        except (TypeError, ValueError, KeyError, json.JSONDecodeError) as error:
            self.stop_robot()
            self.rospy.logerr("Invalid guide plan for safe navigation: %s", error)
            self.publish_state(STATE_FAILED)
            self.publish_result("Plan de navegacion invalido: {}".format(error))
            return

        self.active_plan = plan
        self.display_name = plan.get("display_name") or plan.get("zone_name") or plan.get("zone_id", "destino")
        self.started_at = self.rospy.Time.now().to_sec()
        self.path = []
        self.last_plan_time = 0.0
        self.publish_state(STATE_NAVIGATING)
        self.rospy.loginfo("Safe navigation goal for %s: x=%.3f y=%.3f", self.display_name, self.active_goal[0], self.active_goal[1])

    def get_robot_pose(self):
        try:
            trans, rot = self.tf_listener.lookupTransform(self.global_frame, self.base_frame, self.rospy.Time(0))
        except (
            self.tf.LookupException,
            self.tf.ConnectivityException,
            self.tf.ExtrapolationException,
        ) as error:
            self.rospy.logwarn_throttle(2.0, "Safe navigation waiting for TF pose: %s", error)
            return None
        yaw = self.tf.transformations.euler_from_quaternion(rot)[2]
        return (trans[0], trans[1], yaw)

    def scan_is_fresh(self, now):
        if self.latest_scan is None:
            return False
        stamp = self.latest_scan.header.stamp.to_sec()
        return sensor_is_fresh(stamp, now, self.scan_stale_sec)

    def plan_from_pose(self, robot_pose, now):
        self.path = plan_clearance_path(
            self.navigation_grid,
            (robot_pose[0], robot_pose[1]),
            self.active_goal,
            goal_snap_radius_m=self.goal_snap_radius_m,
        )
        self.last_plan_time = now
        if not self.path:
            self.rospy.logwarn(
                "Safe navigation could not find a clear path to %s from x=%.2f y=%.2f",
                self.display_name,
                robot_pose[0],
                robot_pose[1],
            )
            return False
        self.publish_debug_path()
        self.rospy.loginfo("Safe navigation planned %d waypoints to %s", len(self.path), self.display_name)
        return True

    def publish_debug_path(self):
        path_msg = self.messages.Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = self.rospy.Time.now()
        for x, y in self.path:
            pose = self.messages.PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def finish_success(self):
        self.stop_robot()
        self.rospy.loginfo("Safe navigation to %s succeeded", self.display_name)
        self.publish_state(STATE_SUCCEEDED)
        self.publish_result("Navegacion completada: {}".format(self.display_name))
        self.active_plan = None

    def finish_failed(self, reason):
        self.stop_robot()
        self.rospy.logwarn("Safe navigation to %s failed: %s", self.display_name, reason)
        self.publish_state(STATE_FAILED)
        self.publish_result("Navegacion fallida hacia {}: {}".format(self.display_name, reason))
        self.active_plan = None

    def on_timer(self, _event):
        if self.active_plan is None:
            return

        now = self.rospy.Time.now().to_sec()
        if self.started_at is not None and now - self.started_at > self.navigation_timeout_sec:
            self.finish_failed("timeout")
            return

        if self.navigation_grid is None:
            self.stop_robot()
            self.publish_state(STATE_WAITING_FOR_MAP)
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.stop_robot()
            self.publish_state(STATE_WAITING_FOR_POSE)
            return

        if not self.scan_is_fresh(now):
            self.stop_robot()
            self.publish_state(STATE_WAITING_FOR_SENSOR)
            self.rospy.logwarn_throttle(2.0, "Safe navigation stopped: /scan is stale or missing")
            return

        distance_to_goal = math.hypot(robot_pose[0] - self.active_goal[0], robot_pose[1] - self.active_goal[1])
        if distance_to_goal <= self.goal_tolerance_m:
            self.finish_success()
            return

        if not self.path or now - self.last_plan_time >= self.replan_interval_sec:
            if not self.plan_from_pose(robot_pose, now):
                self.finish_failed("no se encontro trayectoria libre en el mapa")
                return

        front_range = front_min_range(
            self.latest_scan.ranges,
            self.latest_scan.angle_min,
            self.latest_scan.angle_increment,
            self.front_sector_rad,
        )
        if front_range is not None and front_range <= self.emergency_stop_m:
            self.stop_robot()
            self.path = []
            self.publish_state(STATE_BLOCKED)
            self.rospy.logwarn_throttle(1.0, "Safe navigation blocked: obstacle at %.2fm", front_range)
            return

        target = path_to_lookahead(self.path, (robot_pose[0], robot_pose[1]), self.lookahead_m)
        if target is None:
            self.path = []
            return

        desired_yaw = math.atan2(target[1] - robot_pose[1], target[0] - robot_pose[0])
        heading_error = normalize_angle(desired_yaw - robot_pose[2])
        angular = clamp(self.heading_gain * heading_error, -self.max_angular_speed, self.max_angular_speed)
        linear = self.max_linear_speed if abs(heading_error) <= self.heading_tolerance else 0.0

        if front_range is not None and front_range < self.slowdown_front_m:
            scale = clamp(
                (front_range - self.emergency_stop_m) / max(self.slowdown_front_m - self.emergency_stop_m, 1e-6),
                0.0,
                1.0,
            )
            linear *= scale

        twist = self.messages.Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publish_state(STATE_NAVIGATING)
        self.cmd_pub.publish(twist)


def _build_ros_messages():
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    class Messages:
        pass

    messages = Messages()
    messages.PoseStamped = PoseStamped
    messages.Twist = Twist
    messages.OccupancyGrid = OccupancyGrid
    messages.Path = Path
    messages.LaserScan = LaserScan
    messages.String = String
    return messages


def main():
    import rospy
    import tf

    rospy.init_node("safe_navigation_manager_node")
    SafeNavigationManagerNode(rospy, _build_ros_messages(), tf)
    rospy.loginfo("safe_navigation_manager_node ready")
    rospy.spin()


if __name__ == "__main__":
    main()
