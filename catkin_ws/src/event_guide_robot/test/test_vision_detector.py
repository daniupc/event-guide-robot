#!/usr/bin/env python3
import importlib.util
import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DETECTOR_PATH = PACKAGE_ROOT / "scripts" / "vision_detector_node.py"


def load_detector_module():
    spec = importlib.util.spec_from_file_location("vision_detector_node", DETECTOR_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_marker_index_maps_marker_ids_to_label_ids():
    detector = load_detector_module()
    semantic_map = {
        "zones": {
            "north": {
                "labels": {
                    "qualcomm_ai_hub": {"marker_id": 11},
                    "samsung_galaxy": {"marker_id": "12"},
                    "draft_without_marker": {"aliases": ["draft"]},
                }
            },
            "south": {
                "labels": {
                    "ericsson_5g": {"marker_id": 31},
                }
            },
        }
    }

    index = detector.build_marker_index(semantic_map)

    assert index == {11: "qualcomm_ai_hub", 12: "samsung_galaxy", 31: "ericsson_5g"}


def test_stable_detection_becomes_true_after_required_consecutive_frames():
    detector = load_detector_module()
    history = []

    assert detector.stable_detection(11, history, required_frames=3) is False
    assert history == [11]
    assert detector.stable_detection(11, history, required_frames=3) is False
    assert history == [11, 11]
    assert detector.stable_detection(11, history, required_frames=3) is True
    assert history == [11, 11, 11]


def test_stable_detection_resets_when_candidate_id_changes():
    detector = load_detector_module()
    history = []

    detector.stable_detection(11, history, required_frames=3)
    detector.stable_detection(11, history, required_frames=3)
    assert detector.stable_detection(12, history, required_frames=3) is False

    assert history == [12]


def test_detection_to_json_contains_generic_detection_fields_and_stamp():
    detector = load_detector_module()

    payload = json.loads(
        detector.detection_to_json(
            marker_id=11,
            label_id="qualcomm_ai_hub",
            confidence=0.75,
            stable=True,
        )
    )

    assert payload["marker_id"] == 11
    assert payload["label_id"] == "qualcomm_ai_hub"
    assert payload["confidence"] == 0.75
    assert payload["stable"] is True
    assert isinstance(payload["stamp"], float)


def test_default_image_topic_is_turtlebot3_rpicamera_raw_image():
    detector = load_detector_module()

    assert detector.DEFAULT_IMAGE_TOPIC == "/raspicam_node/image"
