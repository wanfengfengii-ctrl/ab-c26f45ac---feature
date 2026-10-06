"""API tests for POST /api/trajectories/audit (in-process via TestClient)."""

import copy

from fastapi.testclient import TestClient

from app.main import app, HEALTH_PATH

client = TestClient(app)
AUDIT = "/api/trajectories/audit"


def base_payload():
    """Two C0+C1-continuous segments, comfortably inside all limits."""
    return {
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


class TestHealth:
    def test_health_endpoint(self):
        resp = client.get(HEALTH_PATH)
        assert resp.status_code == 200
        assert resp.json()["status"] == "ok"


class TestApproval:
    def test_approved_with_exact_peaks(self):
        resp = client.post(AUDIT, json=base_payload())
        assert resp.status_code == 200
        body = resp.json()
        assert body["approved"] is True
        assert body["violations"] == []
        assert body["joints"] == [
            {
                "joint_index": 0,
                "joint": "j1",
                "peak_velocity": "2.4",
                "peak_acceleration": "4.8",
            }
        ]

    def test_equal_to_limit_passes(self):
        payload = {
            "joints": [
                {
                    "id": "j1",
                    "travel": {"min": "-2", "max": "2"},
                    "velocity_limit": "1.5",
                    "acceleration_limit": "3",
                }
            ],
            "segments": [
                {"duration": "1", "control_positions": {"j1": ["0", "0.5", "0.5", "1"]}}
            ],
        }
        body = client.post(AUDIT, json=payload).json()
        assert body["approved"] is True
        assert body["joints"][0]["peak_velocity"] == "1.5"
        assert body["joints"][0]["peak_acceleration"] == "3"

    def test_travel_bounds_are_inclusive(self):
        payload = {
            "joints": [
                {
                    "id": "j1",
                    "travel": {"min": "0", "max": "1"},
                    "velocity_limit": "10",
                    "acceleration_limit": "100",
                }
            ],
            "segments": [
                {"duration": "1", "control_positions": {"j1": ["0", "0", "1", "1"]}}
            ],
        }
        assert client.post(AUDIT, json=payload).json()["approved"] is True

    def test_json_numbers_accepted_and_exact(self):
        payload = base_payload()
        payload["segments"][0]["duration"] = 0.5
        payload["segments"][0]["control_positions"]["j1"] = [0, 0.2, 0.6, 1.0]
        resp = client.post(AUDIT, json=payload)
        assert resp.status_code == 200
        assert resp.json()["joints"][0]["peak_velocity"] == "2.4"

    def test_nonterminating_peak_is_exact(self):
        payload = {
            "joints": [
                {
                    "id": "j1",
                    "travel": {"min": "-1", "max": "1"},
                    "velocity_limit": "10",
                    "acceleration_limit": "10",
                }
            ],
            "segments": [
                {"duration": "0.9", "control_positions": {"j1": ["0", "0", "0.1", "0.1"]}}
            ],
        }
        body = client.post(AUDIT, json=payload).json()
        assert body["approved"] is True
        assert body["joints"][0]["peak_velocity"] == "1/6"
        assert body["joints"][0]["peak_acceleration"] == "20/27"

    def test_velocity_continuity_across_different_durations(self):
        payload = {
            "joints": [
                {
                    "id": "j1",
                    "travel": {"min": "-10", "max": "10"},
                    "velocity_limit": "100",
                    "acceleration_limit": "1000",
                }
            ],
            "segments": [
                {"duration": "0.5", "control_positions": {"j1": ["0", "0", "0.1", "0.2"]}},
                {"duration": "0.3", "control_positions": {"j1": ["0.2", "0.26", "0.3", "0.3"]}},
            ],
        }
        assert client.post(AUDIT, json=payload).json()["approved"] is True


class TestDecimalEquivalence:
    def test_equivalent_spellings_give_identical_responses(self):
        variants = [
            base_payload(),
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
        bodies = []
        for variant in variants:
            resp = client.post(AUDIT, json=variant)
            assert resp.status_code == 200
            bodies.append(resp.json())
        assert bodies[0] == bodies[1] == bodies[2]

    def test_equivalence_holds_when_violations_exist(self):
        def violating(duration, positions):
            return {
                "joints": [
                    {
                        "id": "j1",
                        "travel": {"min": "-0.5", "max": "0.5"},
                        "velocity_limit": "2",
                        "acceleration_limit": "5",
                    }
                ],
                "segments": [{"duration": duration, "control_positions": {"j1": positions}}],
            }

        a = client.post(AUDIT, json=violating("1", ["0", "1", "1", "0"])).json()
        b = client.post(AUDIT, json=violating("1.0", ["0.0", "1.00", "1.0", "0E0"])).json()
        c = client.post(AUDIT, json=violating("10E-1", ["0", "1e0", "1", "0.000"])).json()
        assert a == b == c
        assert a["approved"] is False


class TestViolations:
    def test_all_constraint_types_sorted_stably(self):
        payload = {
            "joints": [
                {
                    "id": "j1",
                    "travel": {"min": "-0.5", "max": "0.5"},
                    "velocity_limit": "2",
                    "acceleration_limit": "5",
                }
            ],
            "segments": [
                {"duration": "1", "control_positions": {"j1": ["0", "1", "1", "0"]}}
            ],
        }
        body = client.post(AUDIT, json=payload).json()
        assert body["approved"] is False
        assert [v["constraint"] for v in body["violations"]] == [
            "travel",
            "velocity",
            "acceleration",
        ]
        travel, velocity, acceleration = body["violations"]
        assert travel["bound"] == "upper"
        assert travel["control_point_index"] == 1
        assert travel["limit"] == "0.5"
        assert travel["value"] == "1"
        assert velocity["limit"] == "2"
        assert velocity["value"] == "3"
        assert acceleration["limit"] == "5"
        assert acceleration["value"] == "6"

    def test_sorted_by_segment_then_joint_then_constraint(self):
        joint = lambda jid: {
            "id": jid,
            "travel": {"min": "-10", "max": "10"},
            "velocity_limit": "1",
            "acceleration_limit": "1000",
        }
        segment = lambda: {"duration": "1", "control_positions": {}}
        payload = {"joints": [joint("j1"), joint("j2")], "segments": [segment(), segment()]}
        for seg in payload["segments"]:
            for jid in ("j1", "j2"):
                seg["control_positions"][jid] = ["0", "0.5", "-0.5", "0"]
        body = client.post(AUDIT, json=payload).json()
        keys = [(v["segment_index"], v["joint_index"], v["constraint"]) for v in body["violations"]]
        assert keys == [
            (0, 0, "velocity"),
            (0, 1, "velocity"),
            (1, 0, "velocity"),
            (1, 1, "velocity"),
        ]

    def test_lower_travel_bound_violation(self):
        payload = {
            "joints": [
                {
                    "id": "j1",
                    "travel": {"min": "-0.5", "max": "2"},
                    "velocity_limit": "100",
                    "acceleration_limit": "1000",
                }
            ],
            "segments": [
                {"duration": "1", "control_positions": {"j1": ["0", "-1", "-1", "0"]}}
            ],
        }
        body = client.post(AUDIT, json=payload).json()
        assert body["approved"] is False
        (violation,) = body["violations"]
        assert violation["constraint"] == "travel"
        assert violation["bound"] == "lower"
        assert violation["limit"] == "-0.5"
        assert violation["value"] == "-1"


class TestUnprocessable:
    def test_position_discontinuity(self):
        payload = base_payload()
        payload["segments"][1]["control_positions"]["j1"][0] = "1.0001"
        resp = client.post(AUDIT, json=payload)
        assert resp.status_code == 422
        detail = resp.json()["detail"]
        assert any(
            e["type"] == "continuity.position"
            and e["loc"] == ["body", "segments", 1, "control_positions", "j1", 0]
            for e in detail
        )

    def test_velocity_discontinuity(self):
        payload = base_payload()
        payload["segments"][1]["control_positions"]["j1"][1] = "1.4001"
        resp = client.post(AUDIT, json=payload)
        assert resp.status_code == 422
        detail = resp.json()["detail"]
        assert any(
            e["type"] == "continuity.velocity"
            and e["loc"] == ["body", "segments", 1, "control_positions", "j1", 1]
            for e in detail
        )

    def test_velocity_continuity_is_exact_not_approximate(self):
        payload = {
            "joints": [
                {
                    "id": "j1",
                    "travel": {"min": "-10", "max": "10"},
                    "velocity_limit": "100",
                    "acceleration_limit": "1000",
                }
            ],
            "segments": [
                {"duration": "0.5", "control_positions": {"j1": ["0", "0", "0.1", "0.2"]}},
                {"duration": "0.3", "control_positions": {"j1": ["0.2", "0.26000000000000001", "0.3", "0.3"]}},
            ],
        }
        resp = client.post(AUDIT, json=payload)
        assert resp.status_code == 422
        assert any(e["type"] == "continuity.velocity" for e in resp.json()["detail"])

    def test_non_positive_duration(self):
        for bad in ("0", "0.0", "-0.5", "-1E-2"):
            payload = base_payload()
            payload["segments"][0]["duration"] = bad
            resp = client.post(AUDIT, json=payload)
            assert resp.status_code == 422, bad
            assert resp.json()["detail"][0]["loc"] == ["body", "segments", 0, "duration"]

    def test_invalid_decimal_notations(self):
        for bad in ("abc", "1/2", "0x10", "NaN", "inf", "-Infinity", "", "1..2", "1_0", "1,5"):
            payload = base_payload()
            payload["segments"][0]["control_positions"]["j1"][0] = bad
            resp = client.post(AUDIT, json=payload)
            assert resp.status_code == 422, bad

    def test_control_positions_must_be_exactly_four(self):
        for bad in (["0", "0.2", "0.6"], ["0", "0.2", "0.6", "1.0", "1.1"], []):
            payload = base_payload()
            payload["segments"][0]["control_positions"]["j1"] = bad
            assert client.post(AUDIT, json=payload).status_code == 422

    def test_missing_joint_in_segment(self):
        payload = base_payload()
        second = copy.deepcopy(payload["joints"][0])
        second["id"] = "j2"
        payload["joints"].append(second)
        for segment in payload["segments"]:
            segment["control_positions"]["j2"] = ["0", "0", "0", "0"]
        payload["segments"][1]["control_positions"] = {"j1": ["1.0", "1.4", "1.6", "1.6"]}
        resp = client.post(AUDIT, json=payload)
        assert resp.status_code == 422
        detail = resp.json()["detail"]
        assert any(
            e["type"] == "value_error.missing_joint"
            and e["loc"] == ["body", "segments", 1, "control_positions"]
            for e in detail
        )

    def test_empty_control_positions_rejected(self):
        payload = base_payload()
        payload["segments"][1]["control_positions"] = {}
        resp = client.post(AUDIT, json=payload)
        assert resp.status_code == 422
        assert resp.json()["detail"][0]["loc"] == ["body", "segments", 1, "control_positions"]

    def test_unknown_joint_in_segment(self):
        payload = base_payload()
        payload["segments"][0]["control_positions"]["j2"] = ["0", "0", "0", "0"]
        resp = client.post(AUDIT, json=payload)
        assert resp.status_code == 422
        assert any(e["type"] == "value_error.unknown_joint" for e in resp.json()["detail"])

    def test_duplicate_joint_id(self):
        payload = base_payload()
        payload["joints"].append(copy.deepcopy(payload["joints"][0]))
        resp = client.post(AUDIT, json=payload)
        assert resp.status_code == 422
        assert any(
            e["type"] == "value_error.duplicate_joint_id" and e["loc"] == ["body", "joints", 1, "id"]
            for e in resp.json()["detail"]
        )

    def test_travel_min_above_max(self):
        payload = base_payload()
        payload["joints"][0]["travel"] = {"min": "2", "max": "-2"}
        assert client.post(AUDIT, json=payload).status_code == 422

    def test_negative_limits_rejected(self):
        payload = base_payload()
        payload["joints"][0]["velocity_limit"] = "-1"
        assert client.post(AUDIT, json=payload).status_code == 422

    def test_unexpected_fields_rejected(self):
        payload = base_payload()
        payload["joints"][0]["name"] = "extra"
        assert client.post(AUDIT, json=payload).status_code == 422


class TestCardinality:
    def test_joint_count_bounds(self):
        payload = base_payload()
        payload["joints"] = []
        assert client.post(AUDIT, json=payload).status_code == 422

        payload = base_payload()
        joints = []
        for i in range(9):
            joint = copy.deepcopy(payload["joints"][0])
            joint["id"] = f"j{i}"
            joints.append(joint)
        payload["joints"] = joints
        assert client.post(AUDIT, json=payload).status_code == 422

    def test_eight_joints_accepted(self):
        joints = []
        control_positions = {}
        for i in range(8):
            jid = f"j{i}"
            joints.append(
                {
                    "id": jid,
                    "travel": {"min": "-1", "max": "1"},
                    "velocity_limit": "1",
                    "acceleration_limit": "1",
                }
            )
            control_positions[jid] = ["0", "0", "0", "0"]
        payload = {
            "joints": joints,
            "segments": [{"duration": "1", "control_positions": control_positions}],
        }
        assert client.post(AUDIT, json=payload).json()["approved"] is True

    def test_segment_count_bounds(self):
        payload = base_payload()
        payload["segments"] = []
        assert client.post(AUDIT, json=payload).status_code == 422

        payload = base_payload()
        payload["segments"] = [
            {"duration": "1", "control_positions": {"j1": ["0", "0", "0", "0"]}}
            for _ in range(201)
        ]
        assert client.post(AUDIT, json=payload).status_code == 422

    def test_two_hundred_segments_accepted(self):
        payload = base_payload()
        payload["segments"] = [
            {"duration": "1", "control_positions": {"j1": ["0", "0", "0", "0"]}}
            for _ in range(200)
        ]
        body = client.post(AUDIT, json=payload).json()
        assert body["approved"] is True
        assert body["joints"][0]["peak_velocity"] == "0"
