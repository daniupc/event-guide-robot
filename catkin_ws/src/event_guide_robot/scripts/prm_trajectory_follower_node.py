#!/usr/bin/env python3
"""Follow PRM-generated trajectories by publishing velocity commands.

This node is the local execution half of the PRM navigation stack. The global
PRM planner publishes a list of poses in ``navigation_path`` and this node
tracks them with a conservative unicycle controller on ``/cmd_vel``.
"""

import json
import math
from types import SimpleNamespace


STATE_IDLE = "IDLE"
STATE_FOLLOWING = "PRM_FOLLOWING_PATH"
STATE_SUCCEEDED = "NAVIGATION_SUCCEEDED"
STATE_FAILED = "NAVIGATION_FAILED"
STATE_BLOCKED = "NAVIGATION_BLOCKED"


class _FallbackTwist:
    """Small stand-in used by tests when geometry_msgs is unavailable."""

    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


def normalize_angle(angle):
    """Normalize ``angle`` to the ``[-pi, pi]`` interval."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def make_stop_command(twist_class=None):
    if twist_class is None:
        twist_class = _FallbackTwist
    return twist_class()


def parse_plan_json(text):
    try:
        payload = json.loads(text)
    except (TypeError, ValueError):
        return None
    return payload if isinstance(payload, dict) else None


def pose_from_amcl(message):
    pose = message.pose.pose
    orientation = pose.orientation
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return {
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "yaw": math.atan2(siny_cosp, cosy_cosp),
    }


def distance_to_waypoint(pose, waypoint):
    return math.hypot(float(waypoint["x"]) - pose["x"], float(waypoint["y"]) - pose["y"])


def scan_has_close_obstacle(scan, safety_distance_m):
    """Return True if a LaserScan-like message has an obstacle too close."""
    for reading in getattr(scan, "ranges", []):
        try:
            distance = float(reading)
        except (TypeError, ValueError):
            continue
        if math.isfinite(distance) and distance < float(safety_distance_m):
            return True
    return False


def compute_tracking_command(pose, waypoint, config, twist_class=None):
    """Compute a Twist-like command to track one PRM waypoint."""
    if twist_class is None:
        twist_class = _FallbackTwist
    twist = twist_class()

    dx = float(waypoint["x"]) - pose["x"]
    dy = float(waypoint["y"]) - pose["y"]
    distance = math.hypot(dx, dy)
    if distance <= float(config["waypoint_tolerance_m"]):
        return twist

    target_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(target_heading - pose["yaw"])
    angular_speed = _clamp(
        heading_error * float(config["angular_gain"]),
        -float(config["max_angular_speed_rad_s"]),
        float(config["max_angular_speed_rad_s"]),
    )
    twist.angular.z = angular_speed

    if abs(heading_error) <= float(config["heading_tolerance_rad"]):
        twist.linear.x = float(config["linear_speed_m_s"])

    return twist


class PrmTrajectoryFollowerNode:
    """ROS adapter for conservative PRM path tracking."""

    def __init__(self, rospy_module, string_msg, twist_msg, pose_msg=object, scan_msg=object):
        self.rospy = rospy_module
        self.string_msg = string_msg
        self.twist_msg = twist_msg

        self.plan_topic = self.rospy.get_param("~plan_topic", "/guide/navigation_plan")
        self.amcl_pose_topic = self.rospy.get_param("~amcl_pose_topic", "/amcl_pose")
        self.scan_topic = self.rospy.get_param("~scan_topic", "/scan")
        self.control_rate_hz = self.rospy.get_param("~control_rate_hz", 10.0)
        self.path_timeout_sec = self.rospy.get_param("~path_timeout_sec", 120.0)
        self.safety_distance_m = self.rospy.get_param("~safety_distance_m", 0.18)
        self.config = {
            "waypoint_tolerance_m": self.rospy.get_param("~waypoint_tolerance_m", 0.15),
            "linear_speed_m_s": self.rospy.get_param("~linear_speed_m_s", 0.12),
            "angular_gain": self.rospy.get_param("~angular_gain", 1.5),
            "max_angular_speed_rad_s": self.rospy.get_param(
                "~max_angular_speed_rad_s", 0.6
            ),
            "heading_tolerance_rad": self.rospy.get_param(
                "~heading_tolerance_rad", 0.25
            ),
        }

        self.state = STATE_IDLE
        self.active_plan = None
        self.navigation_path = []
        self.current_waypoint_index = 0
        self.current_pose = None
        self.path_start_time = None
        self.obstacle_blocked = False

        self.cmd_pub = self.rospy.Publisher("/cmd_vel", twist_msg, queue_size=10)
        self.state_pub = self.rospy.Publisher("/guide/state", string_msg, queue_size=10)
        self.result_pub = self.rospy.Publisher("/guide/result", string_msg, queue_size=10)
        self.plan_sub = self.rospy.Subscriber(
            self.plan_topic, string_msg, self.on_plan
        )
        self.pose_sub = self.rospy.Subscriber(
            self.amcl_pose_topic, pose_msg, self.on_pose
        )
        self.scan_sub = self.rospy.Subscriber(self.scan_topic, scan_msg, self.on_scan)
        period = 1.0 / max(float(self.control_rate_hz), 0.1)
        self.timer = self.rospy.Timer(self.rospy.Duration(period), self.on_timer)
        self.rospy.on_shutdown(self.stop_robot)

    def _now(self):
        return self.rospy.Time.now().to_sec()

    def publish_text(self, publisher, text):
        publisher.publish(self.string_msg(data=text))

    def stop_robot(self):
        self.cmd_pub.publish(make_stop_command(self.twist_msg))

    def _finish(self, state, result):
        self.stop_robot()
        self.state = state
        self.active_plan = None
        self.navigation_path = []
        self.current_waypoint_index = 0
        self.path_start_time = None
        self.publish_text(self.state_pub, state)
        self.publish_text(self.result_pub, result)

    def on_plan(self, message):
        plan = parse_plan_json(message.data)
        if plan is None:
            self.rospy.logwarn("Ignoring malformed PRM navigation plan: %s", message.data)
            self._finish(STATE_FAILED, "Plan PRM malformado")
            return

        if plan.get("status") != "FOUND":
            self.rospy.logwarn("Ignoring unsuccessful semantic plan: %s", plan)
            self._finish(STATE_FAILED, "Plan semantico sin objetivo navegable")
            return

        path = plan.get("navigation_path")
        if not isinstance(path, list) or not path:
            self.rospy.logwarn("Ignoring PRM plan without navigation_path: %s", plan)
            self._finish(STATE_FAILED, "Plan PRM sin trayectoria")
            return

        try:
            self.navigation_path = [
                {
                    "x": float(waypoint["x"]),
                    "y": float(waypoint["y"]),
                    "yaw": float(waypoint.get("yaw", 0.0)),
                }
                for waypoint in path
            ]
        except (KeyError, TypeError, ValueError):
            self.rospy.logwarn("Ignoring PRM plan with invalid waypoint: %s", plan)
            self._finish(STATE_FAILED, "Waypoint PRM invalido")
            return

        self.stop_robot()
        self.active_plan = plan
        self.current_waypoint_index = 0
        self.path_start_time = self._now()
        self.obstacle_blocked = False
        self.state = STATE_FOLLOWING
        self.publish_text(self.state_pub, STATE_FOLLOWING)
        self.rospy.loginfo("Following PRM path with %d waypoint(s)", len(self.navigation_path))

    def on_pose(self, message):
        self.current_pose = pose_from_amcl(message)

    def on_scan(self, message):
        self.obstacle_blocked = scan_has_close_obstacle(message, self.safety_distance_m)

    def _advance_reached_waypoints(self):
        while self.current_waypoint_index < len(self.navigation_path):
            waypoint = self.navigation_path[self.current_waypoint_index]
            if distance_to_waypoint(self.current_pose, waypoint) > float(
                self.config["waypoint_tolerance_m"]
            ):
                break
            self.current_waypoint_index += 1

    def on_timer(self, _event):
        if self.state != STATE_FOLLOWING:
            return

        if self.current_pose is None:
            return

        if self.obstacle_blocked:
            self.rospy.logwarn("PRM follower blocked by a close obstacle")
            self._finish(STATE_BLOCKED, "Navegacion PRM bloqueada por obstaculo")
            return

        if self.path_start_time is not None:
            if (self._now() - self.path_start_time) >= float(self.path_timeout_sec):
                self.rospy.logwarn("PRM follower timed out")
                self._finish(STATE_FAILED, "Timeout siguiendo trayectoria PRM")
                return

        self._advance_reached_waypoints()
        if self.current_waypoint_index >= len(self.navigation_path):
            display_name = self.active_plan.get("display_name", "objetivo")
            self.rospy.loginfo("PRM navigation completed: %s", display_name)
            self._finish(STATE_SUCCEEDED, "Navegacion PRM completada: %s" % display_name)
            return

        waypoint = self.navigation_path[self.current_waypoint_index]
        command = compute_tracking_command(
            self.current_pose, waypoint, self.config, self.twist_msg
        )
        self.cmd_pub.publish(command)


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rospy.init_node("prm_trajectory_follower_node")
    PrmTrajectoryFollowerNode(rospy, String, Twist, PoseWithCovarianceStamped, LaserScan)
    rospy.spin()


if __name__ == "__main__":
    main()
