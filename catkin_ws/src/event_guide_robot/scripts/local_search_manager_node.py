#!/usr/bin/env python3
"""Coordinate local visual search for an already planned guide target.

The module keeps ROS-specific imports inside ``main`` so its pure helpers can be
unit-tested on machines without ROS/catkin installed.
"""

import json
from types import SimpleNamespace


STATE_IDLE = "IDLE"
STATE_SEARCHING = "LOCAL_VISUAL_SEARCH"
STATE_FOUND = "FOUND_TARGET"
STATE_FAILED = "SEARCH_FAILED"


def _has_value(value):
    return value is not None and value != ""


def _coerce_marker_id(value):
    if not _has_value(value):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return value


def target_matches_detection(plan, detection):
    """Return True when a vision detection corresponds to the active plan.

    Marker IDs are the fixed ArUco MVP signal. Label IDs are accepted only as a
    defensive fallback for tests or malformed legacy messages without markers.
    """
    if not isinstance(plan, dict) or not isinstance(detection, dict):
        return False
    if detection.get("stable") is not True:
        return False

    target_marker = _coerce_marker_id(plan.get("marker_id"))
    detected_marker = _coerce_marker_id(detection.get("marker_id"))
    if target_marker is not None and detected_marker is not None:
        return target_marker == detected_marker

    target_label = plan.get("label_id")
    detected_label = detection.get("label_id")
    return _has_value(target_label) and target_label == detected_label


def should_timeout(start_time, now, timeout):
    """Return True once ``timeout`` seconds have elapsed from ``start_time``."""
    return (now - start_time) >= timeout


class _FallbackTwist:
    """Small stand-in used only when geometry_msgs is unavailable."""

    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


def make_twist_command(speed, twist_class=None):
    """Build a Twist-like command rotating around z at ``speed`` rad/s."""
    if twist_class is None:
        twist_class = _FallbackTwist
    twist = twist_class()
    twist.angular.z = float(speed)
    return twist


def parse_detection_json(text):
    """Parse a JSON object detection payload, returning None for bad input."""
    try:
        payload = json.loads(text)
    except (TypeError, ValueError):
        return None
    return payload if isinstance(payload, dict) else None


class LocalSearchManagerNode:
    """ROS adapter that rotates locally until the requested target is detected."""

    def __init__(self, rospy_module, string_msg, twist_msg):
        self.rospy = rospy_module
        self.string_msg = string_msg
        self.twist_msg = twist_msg

        self.rotation_speed = self.rospy.get_param("~rotation_speed_rad_s", 0.25)
        self.search_timeout = self.rospy.get_param("~search_timeout_sec", 45.0)
        self.detection_stale = self.rospy.get_param("~detection_stale_sec", 1.0)
        self.control_rate_hz = self.rospy.get_param("~control_rate_hz", 10.0)

        # Kept as a parameter for launch/config symmetry. The continuous timer
        # loop owns timeout handling so the robot never spins forever.
        self.scan_rotation_duration = self.rospy.get_param(
            "~scan_rotation_duration_sec", 8.0
        )

        self.active_plan = None
        self.pending_plan = None
        self.search_start_time = None
        self.last_detection_time = None
        self.state = STATE_IDLE

        self.cmd_pub = self.rospy.Publisher("/cmd_vel", twist_msg, queue_size=10)
        self.state_pub = self.rospy.Publisher("/guide/state", string_msg, queue_size=10)
        self.result_pub = self.rospy.Publisher("/guide/result", string_msg, queue_size=10)
        self.plan_sub = self.rospy.Subscriber("/guide/plan", string_msg, self.on_plan)
        self.state_sub = self.rospy.Subscriber("/guide/state", string_msg, self.on_state)
        self.detection_sub = self.rospy.Subscriber(
            "/vision/detections", string_msg, self.on_detection
        )
        period = 1.0 / max(float(self.control_rate_hz), 0.1)
        self.timer = self.rospy.Timer(self.rospy.Duration(period), self.on_timer)
        self.rospy.on_shutdown(self.stop_robot)

    def publish_text(self, publisher, text):
        publisher.publish(self.string_msg(data=text))

    def stop_robot(self):
        self.cmd_pub.publish(make_twist_command(0.0, self.twist_msg))

    def _now(self):
        return self.rospy.Time.now().to_sec()

    def on_plan(self, message):
        plan = parse_detection_json(message.data)
        if plan is None:
            self.rospy.logwarn("Ignoring malformed guide plan: %s", message.data)
            return

        if not _has_value(plan.get("label_id")) and not _has_value(plan.get("marker_id")):
            self.rospy.logwarn("Ignoring plan without label_id or marker_id: %s", plan)
            return

        if self.state == STATE_SEARCHING:
            self.stop_robot()

        self.pending_plan = plan
        self.active_plan = None
        self.search_start_time = None
        self.last_detection_time = None
        self.state = STATE_IDLE
        self.rospy.loginfo(
            "Queued local search for label=%s marker=%s until navigation succeeds",
            plan.get("label_id"),
            plan.get("marker_id"),
        )

    def start_search(self, plan):
        self.active_plan = plan
        self.pending_plan = None
        self.search_start_time = self._now()
        self.last_detection_time = None
        self.state = STATE_SEARCHING
        self.publish_text(self.state_pub, STATE_SEARCHING)
        self.rospy.loginfo(
            "Starting local search for label=%s marker=%s",
            plan.get("label_id"),
            plan.get("marker_id"),
        )

    def on_state(self, message):
        if message.data == "NAVIGATION_SUCCEEDED" and self.pending_plan is not None:
            self.start_search(self.pending_plan)
        elif message.data in ("NAVIGATION_FAILED", "PARSE_FAILED"):
            self.pending_plan = None
            self.active_plan = None
            self.search_start_time = None
            self.last_detection_time = None
            if self.state == STATE_SEARCHING:
                self.stop_robot()
            self.state = STATE_IDLE

    def on_detection(self, message):
        if self.state != STATE_SEARCHING or self.active_plan is None:
            return

        detection = parse_detection_json(message.data)
        if detection is None:
            self.rospy.logwarn("Ignoring malformed detection: %s", message.data)
            return

        now = self._now()
        stamp = detection.get("stamp", detection.get("timestamp", now))
        try:
            stamp = float(stamp)
        except (TypeError, ValueError):
            stamp = now

        if (now - stamp) > self.detection_stale:
            self.rospy.logwarn("Ignoring stale detection for local search")
            return

        self.last_detection_time = now
        if target_matches_detection(self.active_plan, detection):
            self.stop_robot()
            self.state = STATE_FOUND
            self.publish_text(self.state_pub, STATE_FOUND)
            result = {
                "status": STATE_FOUND,
                "zone_id": self.active_plan.get("zone_id"),
                "label_id": self.active_plan.get("label_id"),
                "marker_id": self.active_plan.get("marker_id"),
            }
            self.publish_text(self.result_pub, json.dumps(result, sort_keys=True))
            self.rospy.loginfo("Local visual target found: %s", result)

    def on_timer(self, _event):
        if self.state != STATE_SEARCHING or self.active_plan is None:
            return

        now = self._now()
        if should_timeout(self.search_start_time, now, self.search_timeout):
            self.stop_robot()
            self.state = STATE_FAILED
            result = {
                "status": STATE_FAILED,
                "zone_id": self.active_plan.get("zone_id"),
                "label_id": self.active_plan.get("label_id"),
                "marker_id": self.active_plan.get("marker_id"),
                "reason": "local search timeout",
            }
            self.publish_text(self.state_pub, STATE_FAILED)
            self.publish_text(self.result_pub, json.dumps(result, sort_keys=True))
            self.rospy.logwarn("Local visual search timed out: %s", result)
            return

        self.cmd_pub.publish(make_twist_command(self.rotation_speed, self.twist_msg))


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    rospy.init_node("local_search_manager_node")
    LocalSearchManagerNode(rospy, String, Twist)
    rospy.loginfo("local_search_manager_node ready")
    rospy.spin()


if __name__ == "__main__":
    main()
