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


def make_pose_msg(x=0.0, y=0.0, yaw=0.0):
    half_yaw = yaw / 2.0
    return type("FakePoseMsg", (), {
        "pose": type("Cov", (), {
            "pose": type("Pose", (), {
                "position": type("Point", (), {"x": x, "y": y})(),
                "orientation": type("Quat", (), {
                    "x": 0.0,
                    "y": 0.0,
                    "z": math.sin(half_yaw),
                    "w": math.cos(half_yaw),
                })(),
            })()
        })()
    })()


def make_scan(ranges, range_min=0.05, range_max=4.0):
    return type("FakeScan", (), {
        "ranges": ranges,
        "range_min": range_min,
        "range_max": range_max,
    })()


def start_plan(node):
    plan = {
        "status": "FOUND",
        "display_name": "Zona Demo",
        "navigation_path": [{"x": 2.0, "y": 0.0, "yaw": 0.0}],
    }
    node.on_plan(FakeStringMsg(json.dumps(plan)))


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


def test_scan_obstacle_detection_ignores_invalid_and_out_of_range_readings():
    follower = load_follower_module()

    scan = make_scan(
        ranges=[float("nan"), float("inf"), "bad", 0.05, 4.5, 0.12],
        range_min=0.10,
        range_max=4.0,
    )

    assert follower.scan_has_close_obstacle(scan, safety_distance_m=0.18) is True

    scan_without_valid_obstacle = make_scan(
        ranges=[0.05, 4.5, float("nan")],
        range_min=0.10,
        range_max=4.0,
    )

    assert (
        follower.scan_has_close_obstacle(
            scan_without_valid_obstacle,
            safety_distance_m=0.18,
        )
        is False
    )


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
    node.on_pose(make_pose_msg(x=0.0, y=0.0, yaw=0.0))
    node.on_scan(make_scan([1.0, 1.2, 1.4]))
    node.on_timer(None)
    assert node.current_waypoint_index == 1

    node.on_pose(make_pose_msg(x=1.0, y=0.0, yaw=0.0))
    node.on_timer(None)

    assert node.state == follower.STATE_SUCCEEDED
    assert rospy.publishers["/guide/state"].messages[-1].data == "NAVIGATION_SUCCEEDED"
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x == 0.0


def test_follower_waits_safely_until_initial_pose_and_scan_are_available():
    follower = load_follower_module()
    rospy = FakeRospy()
    node = follower.PrmTrajectoryFollowerNode(rospy, FakeStringMsg, follower._FallbackTwist)

    start_plan(node)
    node.on_timer(None)

    assert node.state == follower.STATE_FOLLOWING
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x == 0.0
    assert rospy.publishers["/cmd_vel"].messages[-1].angular.z == 0.0

    node.on_pose(make_pose_msg())
    node.on_timer(None)

    assert node.state == follower.STATE_FOLLOWING
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x == 0.0
    assert rospy.publishers["/cmd_vel"].messages[-1].angular.z == 0.0

    node.on_scan(make_scan([1.0, 1.2, 1.4]))
    node.on_timer(None)

    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x > 0.0


def test_initial_sensor_wait_does_not_consume_path_timeout():
    follower = load_follower_module()
    rospy = FakeRospy()
    rospy.params["~path_timeout_sec"] = 1.0
    rospy.params["~pose_timeout_sec"] = 5.0
    rospy.params["~scan_timeout_sec"] = 5.0
    node = follower.PrmTrajectoryFollowerNode(rospy, FakeStringMsg, follower._FallbackTwist)

    FakeTime.current = 100.0
    start_plan(node)
    FakeTime.current = 101.5
    node.on_pose(make_pose_msg())
    node.on_scan(make_scan([1.0, 1.2, 1.4]))
    node.on_timer(None)

    assert node.state == follower.STATE_FOLLOWING
    assert node.path_start_time == pytest.approx(101.5)
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x > 0.0


def test_follower_stops_during_brief_pose_loss_then_fails_after_timeout():
    follower = load_follower_module()
    rospy = FakeRospy()
    rospy.params["~pose_stale_after_sec"] = 0.2
    rospy.params["~pose_timeout_sec"] = 0.5
    node = follower.PrmTrajectoryFollowerNode(rospy, FakeStringMsg, follower._FallbackTwist)

    FakeTime.current = 100.0
    start_plan(node)
    node.on_pose(make_pose_msg())
    node.on_scan(make_scan([1.0, 1.2, 1.4]))
    node.on_timer(None)
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x > 0.0

    FakeTime.current = 100.3
    node.on_timer(None)
    assert node.state == follower.STATE_FOLLOWING
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x == 0.0

    FakeTime.current = 100.6
    node.on_timer(None)

    assert node.state == follower.STATE_FAILED
    assert rospy.publishers["/guide/result"].messages[-1].data == "Timeout sin pose AMCL reciente"
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x == 0.0


def test_follower_requires_consecutive_close_scans_before_blocking():
    follower = load_follower_module()
    rospy = FakeRospy()
    rospy.params["~obstacle_confirmations_required"] = 2
    node = follower.PrmTrajectoryFollowerNode(rospy, FakeStringMsg, follower._FallbackTwist)

    FakeTime.current = 100.0
    start_plan(node)
    node.on_pose(make_pose_msg())
    node.on_scan(make_scan([float("inf"), 0.12, 1.4]))
    node.on_timer(None)

    assert node.state == follower.STATE_FOLLOWING
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x == 0.0

    node.on_scan(make_scan([0.12, 1.4, 1.5]))
    node.on_timer(None)

    assert node.state == follower.STATE_BLOCKED
    assert rospy.publishers["/guide/state"].messages[-1].data == "NAVIGATION_BLOCKED"
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x == 0.0
