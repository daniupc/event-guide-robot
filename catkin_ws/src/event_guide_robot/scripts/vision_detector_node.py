#!/usr/bin/env python3
"""Camera marker detector for the Event Guide Robot MVP.

The module keeps decision policy outside vision: it publishes ArUco marker
detections as generic JSON and whether the same marker has been seen for enough
consecutive frames. Pure helpers are intentionally ROS-free for local tests.
"""

import json
import math
import time
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover - only relevant on incomplete ROS images
    yaml = None


DEFAULT_REQUIRED_FRAMES = 3
DEFAULT_IMAGE_TOPIC = "/raspicam_node/image"
DEFAULT_DETECTIONS_TOPIC = "/vision/detections"
DEFAULT_MARKER_SIZE_M = 0.16
DEFAULT_HORIZONTAL_FOV_RAD = 1.085595


def build_marker_index(semantic_map):
    """Return a marker_id -> label_id index from a semantic map dictionary.

    Labels without a usable ``marker_id`` are ignored because ArUco IDs are the
    fixed MVP visual signal.
    """
    marker_index = {}
    for zone in semantic_map.get("zones", {}).values():
        for label_id, label in zone.get("labels", {}).items():
            marker_id = label.get("marker_id")
            if marker_id is None:
                continue
            try:
                marker_index[int(marker_id)] = label_id
            except (TypeError, ValueError):
                continue
    return marker_index


def stable_detection(candidate_id, history, required_frames):
    """Update consecutive-frame history and report if ``candidate_id`` is stable.

    ``history`` is a caller-owned mutable list. A changed candidate resets the
    list, which makes this helper independent from ROS timers or image objects.
    """
    required_frames = max(1, int(required_frames))

    if not history or history[-1] == candidate_id:
        history.append(candidate_id)
    else:
        history[:] = [candidate_id]

    if len(history) > required_frames:
        del history[:-required_frames]

    return len(history) >= required_frames and all(
        marker_id == candidate_id for marker_id in history[-required_frames:]
    )


def _point_xy(point):
    """Return an OpenCV corner point as a simple (x, y) tuple."""
    return float(point[0]), float(point[1])


def _distance_between_points(first, second):
    first_x, first_y = _point_xy(first)
    second_x, second_y = _point_xy(second)
    return math.hypot(second_x - first_x, second_y - first_y)


def focal_length_from_width(image_width, horizontal_fov_rad):
    """Estimate focal length in pixels from image width and horizontal FOV."""
    return float(image_width) / (2.0 * math.tan(float(horizontal_fov_rad) / 2.0))


def marker_geometry_from_corners(corners, image_width, marker_size_m, focal_length_px):
    """Estimate marker horizontal offset and distance from its image corners.

    The distance estimate is pinhole-camera based and assumes a printed marker
    with a known physical side length. It is good enough for the final
    short-range approach phase; navigation remains responsible for global
    movement.
    """
    if len(corners) < 4:
        return {}

    points = [_point_xy(point) for point in corners[:4]]
    center_x = sum(point[0] for point in points) / 4.0
    half_width = float(image_width) / 2.0
    center_offset_x = 0.0 if half_width == 0.0 else (center_x - half_width) / half_width

    top_width = _distance_between_points(points[0], points[1])
    bottom_width = _distance_between_points(points[3], points[2])
    marker_width_px = (top_width + bottom_width) / 2.0

    result = {"center_offset_x": center_offset_x}
    if marker_width_px > 0.0 and marker_size_m > 0.0 and focal_length_px > 0.0:
        result["distance_m"] = float(marker_size_m) * float(focal_length_px) / marker_width_px
    return result


def detection_to_json(
    marker_id,
    label_id,
    confidence,
    stable,
    distance_m=None,
    center_offset_x=None,
):
    """Serialize one generic visual detection as JSON."""
    payload = {
        "marker_id": int(marker_id),
        "label_id": label_id,
        "confidence": float(confidence),
        "stable": bool(stable),
        "stamp": time.time(),
    }
    if distance_m is not None:
        payload["distance_m"] = float(distance_m)
    if center_offset_x is not None:
        payload["center_offset_x"] = float(center_offset_x)
    return json.dumps(payload, sort_keys=True)


def load_semantic_map(path):
    """Load a semantic map YAML file for the ROS adapter."""
    if yaml is None:
        raise RuntimeError("PyYAML is required to load the semantic map")
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


class ArucoDetector:
    """Small OpenCV ArUco adapter."""

    def __init__(
        self,
        cv2_module,
        dictionary_name="DICT_4X4_50",
        marker_size_m=DEFAULT_MARKER_SIZE_M,
        horizontal_fov_rad=DEFAULT_HORIZONTAL_FOV_RAD,
    ):
        self.cv2 = cv2_module
        self.aruco = getattr(cv2_module, "aruco", None)
        self.available = self.aruco is not None
        self.dictionary = None
        self.parameters = None
        self.detector = None
        self.marker_size_m = float(marker_size_m)
        self.horizontal_fov_rad = float(horizontal_fov_rad)

        if not self.available:
            return

        dictionary_id = getattr(self.aruco, dictionary_name, None)
        if dictionary_id is None:
            dictionary_id = getattr(self.aruco, "DICT_4X4_50")

        if hasattr(self.aruco, "getPredefinedDictionary"):
            self.dictionary = self.aruco.getPredefinedDictionary(dictionary_id)
        else:
            self.dictionary = self.aruco.Dictionary_get(dictionary_id)

        if hasattr(self.aruco, "DetectorParameters"):
            self.parameters = self.aruco.DetectorParameters()
        else:
            self.parameters = self.aruco.DetectorParameters_create()

        if hasattr(self.aruco, "ArucoDetector"):
            self.detector = self.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect(self, cv_image):
        """Return generic candidate dictionaries for markers in ``cv_image``."""
        if not self.available:
            return []

        if self.detector is not None:
            corners, ids, _rejected = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, _rejected = self.aruco.detectMarkers(
                cv_image, self.dictionary, parameters=self.parameters
            )

        if ids is None:
            return []

        image_width = cv_image.shape[1]
        focal_length_px = focal_length_from_width(image_width, self.horizontal_fov_rad)
        candidates = []
        for marker_id, marker_corners in zip(ids.flatten().tolist(), corners):
            corner_points = marker_corners.reshape((4, 2)).tolist()
            candidate = {"marker_id": int(marker_id), "confidence": 1.0}
            candidate.update(
                marker_geometry_from_corners(
                    corner_points,
                    image_width=image_width,
                    marker_size_m=self.marker_size_m,
                    focal_length_px=focal_length_px,
                )
            )
            candidates.append(candidate)
        return candidates


class VisionDetectorNode:
    """ROS adapter that publishes ArUco detections as JSON."""

    def __init__(self, rospy_module, string_msg, image_msg, bridge, cv2_module):
        self.rospy = rospy_module
        self.string_msg = string_msg
        self.bridge = bridge
        self.cv2 = cv2_module
        self.history = []
        self.warned_unavailable = False

        self.required_frames = int(
            self.rospy.get_param("~required_frames", DEFAULT_REQUIRED_FRAMES)
        )
        self.marker_size_m = float(
            self.rospy.get_param("~marker_size_m", DEFAULT_MARKER_SIZE_M)
        )
        self.horizontal_fov_rad = float(
            self.rospy.get_param("~horizontal_fov_rad", DEFAULT_HORIZONTAL_FOV_RAD)
        )
        image_topic = self.rospy.get_param("~image_topic", DEFAULT_IMAGE_TOPIC)
        default_map = (
            Path(__file__).resolve().parents[1] / "config" / "semantic_map.yaml"
        )
        semantic_map_path = self.rospy.get_param("~semantic_map", str(default_map))
        self.marker_index = build_marker_index(load_semantic_map(semantic_map_path))

        self.detector = (
            ArucoDetector(
                self.cv2,
                marker_size_m=self.marker_size_m,
                horizontal_fov_rad=self.horizontal_fov_rad,
            )
            if self.cv2 is not None
            else None
        )
        if self.detector is None or not self.detector.available:
            self.rospy.logwarn(
                "cv2.aruco is not available; vision_detector_node will not publish detections"
            )
            self.warned_unavailable = True

        self.detections_pub = self.rospy.Publisher(
            DEFAULT_DETECTIONS_TOPIC, string_msg, queue_size=10
        )
        self.image_sub = self.rospy.Subscriber(image_topic, image_msg, self.on_image)

    def on_image(self, message):
        if self.bridge is None or self.detector is None or not self.detector.available:
            if not self.warned_unavailable:
                self.rospy.logwarn(
                    "Vision dependencies unavailable; skipping image without publishing"
                )
                self.warned_unavailable = True
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover - depends on ROS bridge errors
            self.rospy.logwarn("Could not convert camera image: %s", exc)
            return

        for candidate in self.detector.detect(cv_image):
            marker_id = candidate["marker_id"]
            is_stable = stable_detection(marker_id, self.history, self.required_frames)
            label_id = self.marker_index.get(marker_id)
            payload = detection_to_json(
                marker_id=marker_id,
                label_id=label_id,
                confidence=candidate.get("confidence", 1.0),
                stable=is_stable,
                distance_m=candidate.get("distance_m"),
                center_offset_x=candidate.get("center_offset_x"),
            )
            self.detections_pub.publish(self.string_msg(data=payload))


def main():
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    try:
        import cv2
        from cv_bridge import CvBridge
    except ImportError as exc:  # pragma: no cover - depends on ROS environment
        cv2 = None
        bridge = None
        rospy.init_node("vision_detector_node")
        rospy.logwarn("Vision dependencies unavailable: %s", exc)
    else:  # pragma: no cover - exercised on robot/simulation only
        bridge = CvBridge()
        rospy.init_node("vision_detector_node")

    VisionDetectorNode(rospy, String, Image, bridge, cv2)
    rospy.loginfo("vision_detector_node ready")
    rospy.spin()


if __name__ == "__main__":
    main()
