#!/usr/bin/env python3
import importlib.util
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PLANNER_PATH = PACKAGE_ROOT / "scripts" / "semantic_planner_node.py"
SEMANTIC_MAP = PACKAGE_ROOT / "config" / "semantic_map.yaml"


def load_planner_module():
    spec = importlib.util.spec_from_file_location("semantic_planner_node", PLANNER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_resolves_user_request_to_stand_and_zone():
    planner = load_planner_module()
    semantic_map = planner.load_semantic_map(SEMANTIC_MAP)

    result = planner.resolve_command("quiero ir al stand de Qualcomm", semantic_map)

    assert result["status"] == "FOUND"
    assert result["zone_id"] == "zona_arriba"
    assert result["label_id"] == "qualcomm_ai_hub"
    assert result["display_name"] == "Qualcomm AI Hub"
    assert result["marker_id"] == 11
    assert result["nav_goal"]["x"] == -0.529138445854187
    assert result["search_waypoints"]


def test_resolves_request_with_case_and_extra_spaces():
    planner = load_planner_module()
    semantic_map = planner.load_semantic_map(SEMANTIC_MAP)

    result = planner.resolve_command("  LLEVAME   A   nViDiA  ", semantic_map)

    assert result["status"] == "FOUND"
    assert result["zone_id"] == "zona_derecha"
    assert result["label_id"] == "nvidia_edge_ai"


def test_resolves_zone_when_no_specific_stand_is_requested():
    planner = load_planner_module()
    semantic_map = planner.load_semantic_map(SEMANTIC_MAP)

    result = planner.resolve_command("vamos a la zona izquierda", semantic_map)

    assert result["status"] == "FOUND"
    assert result["zone_id"] == "zona_izquierda"
    assert result["label_id"] is None
    assert result["display_name"] == "Zona izquierda"


def test_unknown_request_returns_not_found():
    planner = load_planner_module()
    semantic_map = planner.load_semantic_map(SEMANTIC_MAP)

    result = planner.resolve_command("llevame al stand de cafe", semantic_map)

    assert result == {
        "status": "NOT_FOUND",
        "reason": "No zone or stand alias matched the command",
        "command": "llevame al stand de cafe",
    }
