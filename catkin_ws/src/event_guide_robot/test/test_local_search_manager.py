#!/usr/bin/env python3
import importlib.util
import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LOCAL_SEARCH_PATH = PACKAGE_ROOT / "scripts" / "local_search_manager_node.py"


def load_local_search_module():
    spec = importlib.util.spec_from_file_location(
        "local_search_manager_node", LOCAL_SEARCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_detection_matches_target_marker_id_when_present():
    local_search = load_local_search_module()
    plan = {"label_id": "qualcomm_ai_hub", "marker_id": 11}
    detection = {"label_id": "other", "marker_id": 11, "confidence": 0.92, "stable": True}

    assert local_search.target_matches_detection(plan, detection)


def test_detection_matches_target_label_id_when_marker_id_absent():
    local_search = load_local_search_module()
    plan = {"label_id": "nvidia_edge_ai", "marker_id": None}
    detection = {"label_id": "nvidia_edge_ai", "marker_id": 7, "stable": True}

    assert local_search.target_matches_detection(plan, detection)


def test_detection_does_not_match_different_marker_or_label():
    local_search = load_local_search_module()
    plan = {"label_id": "qualcomm_ai_hub", "marker_id": 11}
    detection = {"label_id": "nvidia_edge_ai", "marker_id": 7, "stable": True}

    assert not local_search.target_matches_detection(plan, detection)


def test_detection_does_not_match_until_vision_reports_stable_confirmation():
    local_search = load_local_search_module()
    plan = {"label_id": "qualcomm_ai_hub", "marker_id": 11}
    detection = {
        "label_id": "qualcomm_ai_hub",
        "marker_id": 11,
        "confidence": 1.0,
        "stable": False,
    }

    assert not local_search.target_matches_detection(plan, detection)


def test_parse_detection_json_accepts_valid_json_and_rejects_invalid_text():
    local_search = load_local_search_module()
    payload = {"label_id": "qualcomm_ai_hub", "marker_id": 11, "confidence": 0.91}

    assert local_search.parse_detection_json(json.dumps(payload)) == payload
    assert local_search.parse_detection_json("not-json") is None
    assert local_search.parse_detection_json("[]") is None


def test_should_timeout_only_after_timeout_elapsed():
    local_search = load_local_search_module()

    assert not local_search.should_timeout(start_time=10.0, now=14.9, timeout=5.0)
    assert local_search.should_timeout(start_time=10.0, now=15.0, timeout=5.0)


def test_make_twist_command_uses_requested_speed_and_zero_default():
    local_search = load_local_search_module()

    class FakeAngular:
        z = 0.0

    class FakeTwist:
        def __init__(self):
            self.angular = FakeAngular()

    rotating = local_search.make_twist_command(0.35, twist_class=FakeTwist)
    stopped = local_search.make_twist_command(0.0, twist_class=FakeTwist)

    assert rotating.angular.z == 0.35
    assert stopped.angular.z == 0.0


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
        self.shutdown_callbacks = []
        self.params = {}
        self.logs = []

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


def test_local_search_waits_for_navigation_success_before_rotating():
    local_search = load_local_search_module()
    rospy = FakeRospy()
    node = local_search.LocalSearchManagerNode(rospy, FakeStringMsg, local_search._FallbackTwist)
    plan = {"status": "FOUND", "label_id": "qualcomm_ai_hub", "marker_id": 11}

    node.on_plan(FakeStringMsg(json.dumps(plan)))
    node.on_timer(None)

    assert node.state == local_search.STATE_IDLE
    assert rospy.publishers["/cmd_vel"].messages == []

    node.on_state(FakeStringMsg("NAVIGATION_SUCCEEDED"))
    node.on_timer(None)

    assert node.state == local_search.STATE_SEARCHING
    assert rospy.publishers["/cmd_vel"].messages[-1].angular.z == node.rotation_speed


def test_approach_command_turns_toward_marker_and_moves_forward_when_centered():
    local_search = load_local_search_module()

    right_marker = {"center_offset_x": 0.5}
    centered_marker = {"center_offset_x": 0.05}

    turning = local_search.make_approach_command(
        right_marker,
        forward_speed=0.08,
        turn_gain=0.6,
        max_turn_speed=0.25,
        center_deadband=0.15,
        twist_class=local_search._FallbackTwist,
    )
    forward = local_search.make_approach_command(
        centered_marker,
        forward_speed=0.08,
        turn_gain=0.6,
        max_turn_speed=0.25,
        center_deadband=0.15,
        twist_class=local_search._FallbackTwist,
    )

    assert turning.angular.z == -0.25
    assert turning.linear.x == 0.0
    assert forward.angular.z < 0.0
    assert forward.linear.x == 0.08


def test_local_search_enters_approach_after_stable_detection_then_finishes_at_target_distance():
    local_search = load_local_search_module()
    rospy = FakeRospy()
    node = local_search.LocalSearchManagerNode(rospy, FakeStringMsg, local_search._FallbackTwist)
    plan = {"status": "FOUND", "label_id": "qualcomm_ai_hub", "marker_id": 11}

    node.on_plan(FakeStringMsg(json.dumps(plan)))
    node.on_state(FakeStringMsg("NAVIGATION_SUCCEEDED"))
    node.on_detection(FakeStringMsg(json.dumps({
        "label_id": "qualcomm_ai_hub",
        "marker_id": 11,
        "stable": True,
        "distance_m": 0.8,
        "center_offset_x": 0.0,
        "stamp": FakeTime.current,
    })))

    assert node.state == local_search.STATE_APPROACHING

    node.on_timer(None)
    assert rospy.publishers["/cmd_vel"].messages[-1].linear.x > 0.0

    node.on_detection(FakeStringMsg(json.dumps({
        "label_id": "qualcomm_ai_hub",
        "marker_id": 11,
        "stable": True,
        "distance_m": 0.2,
        "center_offset_x": 0.0,
        "stamp": FakeTime.current,
    })))

    assert node.state == local_search.STATE_FOUND
