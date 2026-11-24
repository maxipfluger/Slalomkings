
"""Comprehensive tests for GA1 natural-language ROS2 YAML generation."""

import json
import math
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import generate as ga1


# ---------- Defaults and parameter validation ----------

def test_default_values():
    assert ga1.validate_and_merge({}) == {
        "first_side_left": True,
        "throttle": 0.3,
        "offset": 0.5,
        "stop_dist": 0.7,
        "search_steer": 0.8,
        "pass_max_x": 1.5,
        "clear_frames": 6,
    }


def test_partial_changes_preserve_defaults():
    result = ga1.validate_and_merge({
        "first_side_left": False,
        "throttle": 0.27,
    })
    assert result["first_side_left"] is False
    assert result["throttle"] == 0.27
    for name in ga1.DEFAULTS:
        if name not in ("first_side_left", "throttle"):
            assert result[name] == ga1.DEFAULTS[name]


def test_defaults_not_mutated():
    original = ga1.DEFAULTS.copy()
    ga1.validate_and_merge({"throttle": 0.4})
    assert ga1.DEFAULTS == original


@pytest.mark.parametrize("name,bounds", list(ga1.LIMITS.items()))
def test_clamp_all_numeric_parameters(name, bounds):
    lower, upper = bounds

    low_input = -999 if name == "clear_frames" else -999.0
    high_input = 999 if name == "clear_frames" else 999.0

    assert ga1.validate_and_merge({name: low_input})[name] == lower
    assert ga1.validate_and_merge({name: high_input})[name] == upper
    assert ga1.validate_and_merge({name: lower})[name] == lower
    assert ga1.validate_and_merge({name: upper})[name] == upper


def test_boolean_directions():
    assert ga1.validate_and_merge(
        {"first_side_left": True}
    )["first_side_left"] is True

    assert ga1.validate_and_merge(
        {"first_side_left": False}
    )["first_side_left"] is False


@pytest.mark.parametrize("changes", [
    {"unknown_parameter": 1},
    {"cone_count": 4},
    {"cone_spacing": 2.0},
    {"first_side_left": "false"},
    {"first_side_left": 1},
    {"throttle": True},
    {"throttle": "0.25"},
    {"throttle": None},
    {"offset": []},
    {"stop_dist": {}},
    {"search_steer": "fast"},
    {"pass_max_x": None},
    {"clear_frames": 18.5},
    {"clear_frames": "18"},
    {"clear_frames": False},
    {"throttle": float("nan")},
    {"throttle": float("inf")},
    {"offset": float("-inf")},
])
def test_invalid_parameters_rejected(changes):
    with pytest.raises(ValueError):
        ga1.validate_and_merge(changes)


@pytest.mark.parametrize("value", [None, [], "hello", 12, True])
def test_non_dictionary_response_rejected(value):
    with pytest.raises(ValueError):
        ga1.validate_and_merge(value)


# ---------- JSON parsing ----------

def test_valid_json():
    result = ga1.parse_response(
        '{"first_side_left":false,"throttle":0.27}'
    )
    assert result == {
        "first_side_left": False,
        "throttle": 0.27,
    }


@pytest.mark.parametrize("response", [
    "",
    "not json",
    "{broken}",
    '{"throttle":NaN}',
    '{"throttle":Infinity}',
    '{"throttle":-Infinity}',
    '{"throttle":0.3,"throttle":0.4}',
])
def test_invalid_json_rejected(response):
    with pytest.raises(ValueError):
        ga1.parse_response(response)


def test_oversized_response_rejected():
    with pytest.raises(ValueError):
        ga1.parse_response(" " * 10001)


def test_json_overflow_rejected_by_validator():
    changes = ga1.parse_response('{"throttle":1e999}')
    with pytest.raises(ValueError):
        ga1.validate_and_merge(changes)


# ---------- YAML generation ----------

def test_yaml_structure_and_roundtrip(tmp_path):
    params = ga1.validate_and_merge({
        "first_side_left": False,
        "throttle": 0.27,
    })

    output = tmp_path / "genai.yaml"
    text = ga1.save_yaml(params, output)

    assert output.exists()
    assert text == output.read_text(encoding="utf-8")

    document = yaml.safe_load(text)
    assert set(document) == {"driver"}
    assert set(document["driver"]) == {"ros__parameters"}

    loaded = document["driver"]["ros__parameters"]
    assert loaded == params
    assert len(loaded) == 7
    assert type(loaded["first_side_left"]) is bool
    assert type(loaded["clear_frames"]) is int
    assert type(loaded["throttle"]) is float


# ---------- Mock API tests: no network ----------

def test_missing_api_key(monkeypatch):
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)

    with pytest.raises(RuntimeError, match="GEMINI_API_KEY"):
        ga1.call_gemini("Start on the left")


def test_mock_gemini_success(monkeypatch):
    monkeypatch.setenv("GEMINI_API_KEY", "test-key-not-real")

    class FakeModels:
        def generate_content(self, **kwargs):
            assert kwargs["model"] == ga1.MODEL
            assert kwargs["contents"] == "Start on the right"
            assert kwargs["config"]["response_mime_type"] == "application/json"
            return SimpleNamespace(
                text='{"first_side_left":false}'
            )

    class FakeClient:
        def __init__(self):
            self.models = FakeModels()
            self.closed = False

        def close(self):
            self.closed = True

    fake = FakeClient()
    monkeypatch.setattr(ga1.genai, "Client", lambda **kwargs: fake)

    result = ga1.call_gemini("Start on the right")
    assert result == {"first_side_left": False}
    assert fake.closed


def test_mock_api_failure_closes_client(monkeypatch):
    monkeypatch.setenv("GEMINI_API_KEY", "test-key-not-real")

    class BrokenModels:
        def generate_content(self, **kwargs):
            raise RuntimeError("Simulated API failure")

    class FakeClient:
        def __init__(self):
            self.models = BrokenModels()
            self.closed = False

        def close(self):
            self.closed = True

    fake = FakeClient()
    monkeypatch.setattr(ga1.genai, "Client", lambda **kwargs: fake)

    with pytest.raises(RuntimeError, match="Simulated API failure"):
        ga1.call_gemini("Drive slower")

    assert fake.closed


# ---------- CLI workflow without writing actual genai.yaml ----------

def test_main_success(monkeypatch):
    monkeypatch.setattr(
        sys, "argv", ["generate.py", "Drive slower"]
    )
    monkeypatch.setattr(
        ga1, "call_gemini", lambda prompt: {"throttle": 0.25}
    )

    captured = {}

    def fake_save(parameters, path):
        captured["parameters"] = parameters
        captured["path"] = path
        return "Valid YAML"

    monkeypatch.setattr(ga1, "save_yaml", fake_save)

    assert ga1.main() == 0
    assert captured["parameters"]["throttle"] == 0.25
    assert captured["path"].name == "genai.yaml"


def test_main_invalid_response_does_not_write(monkeypatch):
    monkeypatch.setattr(
        sys, "argv", ["generate.py", "Drive faster"]
    )
    monkeypatch.setattr(
        ga1, "call_gemini",
        lambda prompt: {"unsupported_field": 123},
    )

    def forbidden_write(*args):
        pytest.fail("Invalid response must not be written")

    monkeypatch.setattr(ga1, "save_yaml", forbidden_write)
    assert ga1.main() == 1


def test_main_api_failure_does_not_write(monkeypatch):
    monkeypatch.setattr(
        sys, "argv", ["generate.py", "Drive slower"]
    )

    def failed_api(prompt):
        raise RuntimeError("API unavailable")

    monkeypatch.setattr(ga1, "call_gemini", failed_api)
    monkeypatch.setattr(
        ga1, "save_yaml",
        lambda *args: pytest.fail("Must not write after API failure"),
    )

    assert ga1.main() == 1


# ---------- Real API integration tests ----------

LIVE_CASES = [
    (
        "Start on the right, set throttle to 0.27, "
        "4 yellow cones 2 m apart.",
        "explicit",
    ),
    (
        "Start on the left, drive a bit slower, "
        "stop at the pink cone.",
        "qualitative",
    ),
    (
        "4 yellow cones, 2 m apart. "
        "Stop at the pink cone.",
        "unsupported-only",
    ),
]


@pytest.mark.skipif(
    not os.environ.get("GEMINI_API_KEY"),
    reason="GEMINI_API_KEY not configured",
)
@pytest.mark.parametrize("prompt,case", LIVE_CASES)
def test_real_gemini_end_to_end(prompt, case, tmp_path):
    changes = ga1.call_gemini(prompt)
    params = ga1.validate_and_merge(changes)

    output = tmp_path / "genai.yaml"
    text = ga1.save_yaml(params, output)

    loaded = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert loaded["driver"]["ros__parameters"] == params
    assert set(params) == set(ga1.DEFAULTS)

    print(f"\n=== LIVE API: {case} ===")
    print("Prompt:", prompt)
    print("LLM changes:", changes)
    print(text)

    if case == "explicit":
        assert params["first_side_left"] is False
        assert params["throttle"] == pytest.approx(0.27)
        assert set(changes) == {"first_side_left", "throttle"}

    elif case == "qualitative":
        assert params["first_side_left"] is True
        assert 0.20 <= params["throttle"] < 0.30
        assert params["stop_dist"] == 0.7
        assert set(changes) <= {"first_side_left", "throttle"}

    elif case == "unsupported-only":
        assert changes == {}
        assert params == ga1.DEFAULTS
