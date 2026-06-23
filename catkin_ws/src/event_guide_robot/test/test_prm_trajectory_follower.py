#!/usr/bin/env python3
import importlib.util
import json
import math
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FOLLOWER_PATH = PACKAGE_ROOT / "scripts" / "prm_trajectory_follower_node.py"


def load_follower_module():
    spec = importlib.util.spec_from_file_location("prm_trajectory_follower_node", FOLLOWER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeStringMsg:
    def __init__(self, data=""):
        self.data = data


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeTimer:
    def __init__(self, duration, callback):
        self.duration = duration
        self.callback = callback


class FakeTimeValue:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class FakeTime:
    current = 100.0

    @classmethod
    def now(cls):
        return FakeTimeValue(cls.current)


class FakeRospy:
    Time = FakeTime

    def __init__(self):
        self.publishers = {}
        self.subscribers = []
        self.timers = []
        self.params = {}
        self.logs = []
        self.shutdown_callbacks = []

    def get_param(self, name, default):
        return self.params.get(name, default)

    def Publisher(self, topic, _msg_type, queue_size=10):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    def Subscriber(self, topic, _msg_type, callback):
        self.subscribers.append((topic, callback))
        return (topic, callback)

    def Timer(self, duration, callback):
        timer = FakeTimer(duration, callback)
        self.timers.append(timer)
        return timer

    def Duration(self, seconds):
        return seconds

    def on_shutdown(self, callback):
        self.shutdown_callbacks.append(callback)

    def loginfo(self, *args):
        self.logs.append(("info", args))

    def logwarn(self, *args):
        self.logs.append(("warn", args))

    def logerr(self, *args):
        self.logs.append(("err", args))


def test_normalize_angle_wraps_to_minus_pi_pi():
    follower = load_follower_module()

    assert follower.normalize_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert follower.normalize_angle(-math.pi - 0.2) == pytest.approx(math.pi - 0.2)


def test_command_to_waypoint_rotates_before_moving_when_heading_error_is_large():
    follower = load_follower_module()

    command = follower.compute_tracking_command(
        pose={"x": 0.0, "y": 0.0, "yaw": math.pi / 2.0},
        waypoint={"x": 1.0, "y": 0.0, "yaw": 0.0},
        config={
            "linear_speed_m_s": 0.2,
            "angular_gain": 1.5,
            "max_angular_speed_rad_s": 0.6,
            "heading_tolerance_rad": 0.25,
            "waypoint_tolerance_m": 0.12,
        },
        twist_class=follower._FallbackTwist,
    )

    assert command.linear.x == 0.0
    assert command.angular.z == pytest.approx(-0.6)


def test_command_to_waypoint_moves_forward_when_aligned():
    follower = load_follower_module()

    command = follower.compute_tracking_command(
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        waypoint={"x": 1.0, "y": 0.0, "yaw": 0.0},
        config={
            "linear_speed_m_s": 0.2,
            "angular_gain": 1.5,
            "max_angular_speed_rad_s": 0.6,
            "heading_tolerance_rad": 0.25,
            "waypoint_tolerance_m": 0.12,
        },
        twist_class=follower._FallbackTwist,
    )

    assert command.linear.x == 0.2
    assert command.angular.z == 0.0


def test_follower_advances_waypoints_and_reports_navigation_success():
    follower = load_follower_module()
    rospy = FakeRospy()
    node = follower.PrmTrajectoryFollowerNode(rospy, FakeStringMsg, follower._FallbackTwist)
    plan = {
        "status": "FOUND",
        "display_name": "Zona Demo",
        "navigation_path": [
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            {"x": 1.0, "y": 0.0, "yaw": 0.0},
        ],
    }

    node.on_plan(FakeStringMsg(json.dumps(plan)))
    node.current_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    node.on_timer(None)
    assert node.current_waypoint_index == 1

    node.current_pose = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    node.on_timer(None)

    assert node.state == follower.STATE_SUCCEEDED
    assert rospy.publishers["/guide/state"].messages[-1].data == "NAVIGATION_SUCCEEDED"
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x == 0.0


def test_follower_stops_when_scan_reports_close_obstacle():
    follower = load_follower_module()
    rospy = FakeRospy()
    node = follower.PrmTrajectoryFollowerNode(rospy, FakeStringMsg, follower._FallbackTwist)
    plan = {
        "status": "FOUND",
        "navigation_path": [{"x": 2.0, "y": 0.0, "yaw": 0.0}],
    }
    scan = type("FakeScan", (), {"ranges": [float("inf"), 0.12, 1.4]})()

    node.on_plan(FakeStringMsg(json.dumps(plan)))
    node.on_pose(type("FakePoseMsg", (), {
        "pose": type("Cov", (), {
            "pose": type("Pose", (), {
                "position": type("Point", (), {"x": 0.0, "y": 0.0})(),
                "orientation": type("Quat", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})(),
            })()
        })()
    })())
    node.on_scan(scan)
    node.on_timer(None)

    assert node.state == follower.STATE_BLOCKED
    assert rospy.publishers["/guide/state"].messages[-1].data == "NAVIGATION_BLOCKED"
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x == 0.0
