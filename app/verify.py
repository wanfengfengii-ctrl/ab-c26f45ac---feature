"""One-shot verification service.

Waits for the API to become healthy, then runs, in order:

1. code tests (pytest unit/API suite),
2. build checks (byte-compile, application import, OpenAPI generation),
3. API smoke tests over HTTP against the live service — an approvable
   trajectory, a limit-violating trajectory, decimal-equivalence invariance
   and 422 behaviour,
4. retime smoke tests over HTTP — minimal-scale retiming, exact integer
   cycle counts, decimal-equivalence invariance, 409 conflict reason codes
   and 422 behaviour.

Exits 0 only if every check passes; otherwise exits 1.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
from typing import Callable, List, Tuple

BASE_URL = os.environ.get("API_BASE_URL", "http://127.0.0.1:8000").rstrip("/")
HEALTH_PATH = os.environ.get("API_HEALTH_PATH", "/api/health")
AUDIT_PATH = "/api/trajectories/audit"
RETIME_PATH = "/api/trajectories/retime"
APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HEALTH_TIMEOUT = float(os.environ.get("VERIFY_HEALTH_TIMEOUT", "90"))

# A trajectory well inside every limit. Two segments, C0+C1 continuous:
# segment 0 ends at 1.0 with velocity 3*(1.0-0.6)/0.5 = 2.4 and segment 1
# starts at 1.0 with velocity 3*(1.4-1.0)/0.5 = 2.4.
PASSING_PAYLOAD = {
    "joints": [
        {
            "id": "j1",
            "travel": {"min": "-2", "max": "2"},
            "velocity_limit": "3",
            "acceleration_limit": "20",
        }
    ],
    "segments": [
        {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
        {"duration": "0.5", "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"]}},
    ],
}

# Same trajectory in decimal-equivalent spellings: results must be identical.
PASSING_VARIANTS = [
    PASSING_PAYLOAD,
    {
        "joints": [
            {
                "id": "j1",
                "travel": {"min": "-2.0", "max": "2.00"},
                "velocity_limit": "3.0",
                "acceleration_limit": "20.00",
            }
        ],
        "segments": [
            {"duration": "0.50", "control_positions": {"j1": ["0.0", "0.20", "0.60", "1.00"]}},
            {"duration": "0.500", "control_positions": {"j1": ["1.000", "1.40", "1.60", "1.600"]}},
        ],
    },
    {
        "joints": [
            {
                "id": "j1",
                "travel": {"min": "-2E0", "max": "2e0"},
                "velocity_limit": "30E-1",
                "acceleration_limit": "2E1",
            }
        ],
        "segments": [
            {"duration": "5E-1", "control_positions": {"j1": ["0E0", "2E-1", "6E-1", "1"]}},
            {"duration": "500E-3", "control_positions": {"j1": ["1.0", "14E-1", "16E-1", "1.6"]}},
        ],
    },
]

# One segment violating all three constraint types: control positions leave
# the travel interval, peak velocity 3 > 2, peak acceleration 6 > 5.
VIOLATING_PAYLOAD = {
    "joints": [
        {
            "id": "j1",
            "travel": {"min": "-0.5", "max": "0.5"},
            "velocity_limit": "2",
            "acceleration_limit": "5",
        }
    ],
    "segments": [
        {"duration": "1", "control_positions": {"j1": ["0", "1", "1", "0"]}},
    ],
}

# Position discontinuity between the two segments (1.0 vs 1.0001).
DISCONTINUOUS_PAYLOAD = {
    "joints": [
        {
            "id": "j1",
            "travel": {"min": "-2", "max": "2"},
            "velocity_limit": "3",
            "acceleration_limit": "20",
        }
    ],
    "segments": [
        {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
        {"duration": "0.5", "control_positions": {"j1": ["1.0001", "1.4", "1.6", "1.6"]}},
    ],
}

# Non-positive duration.
BAD_DURATION_PAYLOAD = {
    "joints": [
        {
            "id": "j1",
            "travel": {"min": "-2", "max": "2"},
            "velocity_limit": "3",
            "acceleration_limit": "20",
        }
    ],
    "segments": [
        {"duration": "0", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
    ],
}


def _retime_payload(**overrides):
    """The passing trajectory plus retiming parameters.

    Cycle 0.2 makes 0.5 s segments land on 5/2 cycles, so only even scales
    keep whole cycles; the limits are already satisfied at scale 1, hence the
    smallest qualifying scale is exactly 2.
    """
    payload = json.loads(json.dumps(PASSING_PAYLOAD))
    payload["cycle_duration"] = "0.2"
    payload["max_scale"] = 1000
    payload.update(overrides)
    return payload


# Same retime request in decimal-equivalent spellings: identical results.
RETIME_VARIANTS = [
    _retime_payload(),
    _retime_payload(
        joints=[
            {
                "id": "j1",
                "travel": {"min": "-2.0", "max": "2.00"},
                "velocity_limit": "3.0",
                "acceleration_limit": "20.00",
            }
        ],
        segments=[
            {"duration": "0.50", "control_positions": {"j1": ["0.0", "0.20", "0.60", "1.00"]}},
            {"duration": "0.500", "control_positions": {"j1": ["1.000", "1.40", "1.60", "1.600"]}},
        ],
        cycle_duration="0.20",
        max_scale="1000",
    ),
    _retime_payload(
        joints=[
            {
                "id": "j1",
                "travel": {"min": "-2E0", "max": "2e0"},
                "velocity_limit": "30E-1",
                "acceleration_limit": "2E1",
            }
        ],
        segments=[
            {"duration": "5E-1", "control_positions": {"j1": ["0E0", "2E-1", "6E-1", "1"]}},
            {"duration": "500E-3", "control_positions": {"j1": ["1.0", "14E-1", "16E-1", "1.6"]}},
        ],
        cycle_duration="2E-1",
    ),
]


def wait_for_health() -> None:
    url = BASE_URL + HEALTH_PATH
    deadline = time.monotonic() + HEALTH_TIMEOUT
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=5) as resp:
                if resp.status == 200:
                    print(f"[ ok ] API healthy at {url}")
                    return
        except Exception:
            pass
        time.sleep(1)
    raise RuntimeError(f"API did not become healthy at {url} within {HEALTH_TIMEOUT:.0f}s")


def http_post(path: str, payload: dict) -> Tuple[int, dict]:
    request = urllib.request.Request(
        BASE_URL + path,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=15) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def run_unit_tests() -> None:
    proc = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", "tests"],
        cwd=APP_DIR,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"pytest exited with code {proc.returncode}")


def run_build_checks() -> None:
    proc = subprocess.run(
        [sys.executable, "-m", "compileall", "-q", "app", "tests"],
        cwd=APP_DIR,
    )
    if proc.returncode != 0:
        raise RuntimeError("byte-compilation of app/tests failed")
    proc = subprocess.run(
        [
            sys.executable,
            "-c",
            "from app.main import app\n"
            "schema = app.openapi()\n"
            "assert '/api/trajectories/audit' in schema['paths']\n"
            "assert '/api/trajectories/retime' in schema['paths']\n",
        ],
        cwd=APP_DIR,
    )
    if proc.returncode != 0:
        raise RuntimeError("application import / OpenAPI generation failed")


def smoke_approved() -> None:
    status, body = http_post(AUDIT_PATH, PASSING_PAYLOAD)
    assert status == 200, f"expected 200, got {status}: {body}"
    assert body["approved"] is True, f"expected approval: {body}"
    assert body["violations"] == [], f"expected no violations: {body}"
    peaks = body["joints"][0]
    assert peaks["peak_velocity"] == "2.4", peaks
    assert peaks["peak_acceleration"] == "4.8", peaks


def smoke_violating() -> None:
    status, body = http_post(AUDIT_PATH, VIOLATING_PAYLOAD)
    assert status == 200, f"expected 200, got {status}: {body}"
    assert body["approved"] is False, f"expected rejection: {body}"
    constraints = [v["constraint"] for v in body["violations"]]
    assert constraints == ["travel", "velocity", "acceleration"], (
        f"violations not in stable constraint order: {constraints}"
    )
    values = [v["value"] for v in body["violations"]]
    assert values == ["1", "3", "6"], f"unexpected exact peaks: {values}"


def smoke_decimal_equivalence() -> None:
    bodies = []
    for variant in PASSING_VARIANTS:
        status, body = http_post(AUDIT_PATH, variant)
        assert status == 200, f"expected 200, got {status}: {body}"
        bodies.append(body)
    for i, body in enumerate(bodies[1:], start=1):
        assert body == bodies[0], (
            f"decimal-equivalent variant {i} changed the audit result:\n"
            f"{json.dumps(bodies[0], sort_keys=True)}\nvs\n{json.dumps(body, sort_keys=True)}"
        )


def smoke_unprocessable() -> None:
    status, body = http_post(AUDIT_PATH, DISCONTINUOUS_PAYLOAD)
    assert status == 422, f"expected 422 for discontinuity, got {status}: {body}"
    detail = body.get("detail")
    assert isinstance(detail, list) and detail, f"missing error detail: {body}"
    for error in detail:
        assert "loc" in error and "msg" in error and "type" in error, error
    assert any(e["type"] == "continuity.position" for e in detail), detail

    status, body = http_post(AUDIT_PATH, BAD_DURATION_PAYLOAD)
    assert status == 422, f"expected 422 for non-positive duration, got {status}: {body}"
    assert isinstance(body.get("detail"), list) and body["detail"], body


def smoke_retime_approved() -> None:
    status, body = http_post(RETIME_PATH, _retime_payload())
    assert status == 200, f"expected 200, got {status}: {body}"
    # Only even scales land 0.5 s segments on whole 0.2 s cycles -> scale 2.
    assert body["scale"] == 2, body
    assert [s["duration"] for s in body["segments"]] == ["1", "1"], body
    assert [s["cycles"] for s in body["segments"]] == [5, 5], body
    assert body["total_duration"] == "2" and body["total_cycles"] == 10, body
    peaks = body["joints"][0]
    assert peaks["peak_velocity"] == "1.2", peaks  # 2.4 / 2
    assert peaks["peak_acceleration"] == "1.2", peaks  # 4.8 / 2**2
    # Minimality certificate: feasible scales are multiples of cycle_multiple
    # that are >= limit_min_scale; the returned scale is the least of them.
    proof = body["proof"]
    assert proof["cycle_multiple"] == 2 and proof["limit_min_scale"] == 1, proof
    assert proof["minimal"] is True, proof
    assert body["scale"] - proof["cycle_multiple"] < proof["limit_min_scale"], body


def smoke_retime_limit_driven() -> None:
    # Velocity limit 1 forces k >= ceil(2.4 / 1) = 3; durations already on
    # whole 0.5 s cycles, so the limit alone drives the scale.
    payload = _retime_payload(cycle_duration="0.5")
    payload["joints"][0]["velocity_limit"] = "1"
    status, body = http_post(RETIME_PATH, payload)
    assert status == 200, f"expected 200, got {status}: {body}"
    assert body["scale"] == 3, body
    assert [s["cycles"] for s in body["segments"]] == [3, 3], body
    peaks = body["joints"][0]
    assert peaks["peak_velocity"] == "0.8", peaks  # 2.4 / 3
    assert peaks["peak_acceleration"] == "8/15", peaks  # 4.8 / 9, exact
    assert body["proof"] == {"cycle_multiple": 1, "limit_min_scale": 3, "minimal": True}


def smoke_retime_decimal_equivalence() -> None:
    bodies = []
    for variant in RETIME_VARIANTS:
        status, body = http_post(RETIME_PATH, variant)
        assert status == 200, f"expected 200, got {status}: {body}"
        bodies.append(body)
    for i, body in enumerate(bodies[1:], start=1):
        assert body == bodies[0], (
            f"decimal-equivalent retime variant {i} changed the result:\n"
            f"{json.dumps(bodies[0], sort_keys=True)}\nvs\n{json.dumps(body, sort_keys=True)}"
        )


def smoke_retime_conflicts() -> None:
    # Travel violations cannot be fixed by slowing down.
    payload = _retime_payload()
    payload["segments"][0]["control_positions"]["j1"][1] = "5"
    status, body = http_post(RETIME_PATH, payload)
    assert status == 409, f"expected 409, got {status}: {body}"
    assert body["detail"]["reason"] == "travel_out_of_bounds", body

    # A non-zero peak can never meet a zero limit.
    for field, reason in (
        ("velocity_limit", "zero_velocity_limit"),
        ("acceleration_limit", "zero_acceleration_limit"),
    ):
        payload = _retime_payload()
        payload["joints"][0][field] = "0"
        status, body = http_post(RETIME_PATH, payload)
        assert status == 409, f"expected 409, got {status}: {body}"
        assert body["detail"]["reason"] == reason, body

    # The smallest qualifying scale (2) exceeds max_scale (1).
    status, body = http_post(RETIME_PATH, _retime_payload(max_scale=1))
    assert status == 409, f"expected 409, got {status}: {body}"
    detail = body["detail"]
    assert detail["reason"] == "scale_exceeds_max", body
    assert detail["required_scale"] == "2" and detail["max_scale"] == 1, body


def smoke_retime_unprocessable() -> None:
    # Non-positive control cycle.
    status, body = http_post(RETIME_PATH, _retime_payload(cycle_duration="0"))
    assert status == 422, f"expected 422 for zero cycle, got {status}: {body}"
    # Out-of-range max_scale.
    for bad in (0, 1000001):
        status, body = http_post(RETIME_PATH, _retime_payload(max_scale=bad))
        assert status == 422, f"expected 422 for max_scale={bad}, got {status}: {body}"
    # Continuity errors stay 422 on the retime endpoint too.
    payload = _retime_payload()
    payload["segments"][1]["control_positions"]["j1"][0] = "1.0001"
    status, body = http_post(RETIME_PATH, payload)
    assert status == 422, f"expected 422 for discontinuity, got {status}: {body}"
    assert any(e["type"] == "continuity.position" for e in body["detail"]), body


def main() -> int:
    checks: List[Tuple[str, Callable[[], None]]] = [
        ("code tests (pytest)", run_unit_tests),
        ("build checks (compileall, import, OpenAPI)", run_build_checks),
        ("API smoke: approvable trajectory", smoke_approved),
        ("API smoke: limit-violating trajectory", smoke_violating),
        ("API smoke: decimal-equivalence invariance", smoke_decimal_equivalence),
        ("API smoke: 422 for discontinuity / non-positive duration", smoke_unprocessable),
        ("API smoke: retime minimal scale and exact cycles", smoke_retime_approved),
        ("API smoke: retime driven by velocity limit", smoke_retime_limit_driven),
        ("API smoke: retime decimal-equivalence invariance", smoke_retime_decimal_equivalence),
        ("API smoke: retime 409 conflict reason codes", smoke_retime_conflicts),
        ("API smoke: retime 422 for bad cycle / max_scale / continuity", smoke_retime_unprocessable),
    ]

    print(f"verify: waiting for API health at {BASE_URL}{HEALTH_PATH}")
    try:
        wait_for_health()
    except RuntimeError as exc:
        print(f"[FAIL] {exc}")
        return 1

    failures = 0
    for name, check in checks:
        try:
            check()
        except Exception as exc:  # noqa: BLE001 - report and continue
            failures += 1
            print(f"[FAIL] {name}: {exc}")
        else:
            print(f"[ ok ] {name}")

    if failures:
        print(f"verify: {failures} of {len(checks)} check(s) failed")
        return 1
    print(f"verify: all {len(checks)} checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
