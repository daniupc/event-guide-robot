#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SEMANTIC_MAP = PACKAGE_ROOT / "config" / "semantic_map.yaml"
WORLD_FILE = PACKAGE_ROOT / "worlds" / "event_guide_world.world"
MATERIAL_FILE = PACKAGE_ROOT / "media" / "materials" / "scripts" / "event_guide_markers.material"
TEXTURE_DIR = PACKAGE_ROOT / "media" / "materials" / "textures"


def semantic_marker_ids():
    text = SEMANTIC_MAP.read_text(encoding="utf-8")
    return sorted({int(match) for match in re.findall(r"marker_id:\s*(\d+)", text)})


class GazeboSimulationAssetsTest(unittest.TestCase):
    def test_world_contains_a_stand_model_for_each_semantic_marker(self):
        world = WORLD_FILE.read_text(encoding="utf-8")

        for marker_id in semantic_marker_ids():
            self.assertIn(f"stand_marker_{marker_id}", world)
            self.assertIn(f"EventGuide/Aruco{marker_id}", world)

    def test_materials_and_textures_exist_for_each_semantic_marker(self):
        material = MATERIAL_FILE.read_text(encoding="utf-8")

        for marker_id in semantic_marker_ids():
            texture = TEXTURE_DIR / f"aruco_{marker_id}.png"
            self.assertTrue(texture.exists(), f"Missing texture for marker {marker_id}")
            self.assertEqual(texture.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            self.assertIn(f"EventGuide/Aruco{marker_id}", material)
            self.assertIn(f"aruco_{marker_id}.png", material)


if __name__ == "__main__":
    unittest.main()
