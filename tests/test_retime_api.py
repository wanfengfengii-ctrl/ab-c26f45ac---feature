"""API tests for POST /api/trajectories/retime (in-process via TestClient)."""

from fastapi.testclient import TestClient

from app.main import app

client = TestClient(app)
RETRIME = "/api/trajectories/retime"


def base_payload(cycle="0.1", max_scale=1_000_000, **overrides):
    payload = {
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
        "cycle_duration": cycle,
        "max_scale": max_scale,
    }
    payload.update(overrides)
    return payload


class TestSuccess:
    def test_aligned_schedule_scale_one(self):
        resp = client.post(RETRIME, json=base_payload())
        assert resp.status_code == 200, resp.text
        body = resp.json()
        assert body["scale"] == 1
        assert body["cycle_duration"] == "0.1"
        assert body["segments"] == [
            {"segment_index": 0, "duration": "0.5", "cycles": 5},
            {"segment_index": 1, "duration": "0.5", "cycles": 5},
        ]
        assert body["joints"] == [
            {"joint_index": 0, "joint": "j1",
             "peak_velocity": "2.4", "peak_acceleration": "4.8"}
        ]

    def test_slowdown_schedule(self):
        resp = client.post(RETRIME, json=base_payload(cycle="0.3"))
        assert resp.status_code == 200, resp.text
        body = resp.json()
        assert body["scale"] == 3
        assert body["segments"] == [
            {"segment_index": 0, "duration": "1.5", "cycles": 5},
            {"segment_index": 1, "duration": "1.5", "cycles": 5},
        ]
        # Velocity divides by 3, acceleration by 9.
        assert body["joints"][0]["peak_velocity"] == "0.8"
        assert body["joints"][0]["peak_acceleration"] == "8/15"

    def test_limit_driven_scale(self):
        payload = base_payload(cycle="0.5")
        payload["joints"][0]["velocity_limit"] = "1.5"
        payload["joints"][0]["acceleration_limit"] = "20"
        body = client.post(RETRIME, json=payload).json()
        assert body["scale"] == 2
        assert all(s["cycles"] == 2 for s in body["segments"])

    def test_boundary_equal_to_limit_scales_down_to_one(self):
        # cycle 0.5 divides 0.5; peaks equal the limits exactly -> scale 1.
        payload = base_payload(cycle="0.5")
        payload["joints"][0]["velocity_limit"] = "2.4"
        payload["joints"][0]["acceleration_limit"] = "4.8"
        body = client.post(RETRIME, json=payload).json()
        assert body["scale"] == 1

    def test_stationary_zero_limits(self):
        payload = base_payload()
        payload["joints"][0]["velocity_limit"] = "0"
        payload["joints"][0]["acceleration_limit"] = "0"
        for seg in payload["segments"]:
            seg["control_positions"]["j1"] = ["1.5", "1.5", "1.5", "1.5"]
        resp = client.post(RETRIME, json=payload)
        assert resp.status_code == 200, resp.text
        assert resp.json()["scale"] == 1

    def test_max_scale_boundary_accepted(self):
        body = client.post(RETRIME, json=base_payload(max_scale=1)).json()
        assert body["scale"] == 1

    def test_json_numbers_accepted(self):
        payload = base_payload()
        payload["cycle_duration"] = 0.1
        resp = client.post(RETRIME, json=payload)
        assert resp.status_code == 200, resp.text
        assert resp.json()["scale"] == 1


class TestConflict409:
    def test_travel_conflict(self):
        payload = base_payload()
        payload["joints"][0]["travel"] = {"min": "-0.5", "max": "0.5"}
        resp = client.post(RETRIME, json=payload)
        assert resp.status_code == 409
        body = resp.json()
        assert body["reason"] == "travel_out_of_bounds"
        assert isinstance(body["msg"], str) and body["msg"]

    def test_zero_velocity_limit_conflict(self):
        payload = base_payload()
        payload["joints"][0]["velocity_limit"] = "0"
        resp = client.post(RETRIME, json=payload)
        assert resp.status_code == 409
        assert resp.json()["reason"] == "zero_limit_with_motion"

    def test_zero_acceleration_limit_conflict(self):
        payload = base_payload()
        payload["joints"][0]["acceleration_limit"] = "0"
        resp = client.post(RETRIME, json=payload)
        assert resp.status_code == 409
        assert resp.json()["reason"] == "zero_limit_with_motion"

    def test_scale_exceeds_max_conflict(self):
        # cycle 0.3 forces scale >= 3; max_scale 2 cannot work.
        resp = client.post(RETRIME, json=base_payload(cycle="0.3", max_scale=2))
        assert resp.status_code == 409
        body = resp.json()
        assert body["reason"] == "scale_exceeds_max"
        assert "3" in body["msg"]

    def test_conflict_reason_is_stable_across_equivalent_spellings(self):
        def call(cycle):
            return client.post(RETRIME, json=base_payload(cycle=cycle, max_scale=2))

        statuses = [call(c).status_code for c in ("0.3", "0.30", "3E-1")]
        reasons = [call(c).json()["reason"] for c in ("0.3", "0.30", "3E-1")]
        assert statuses == [409, 409, 409]
        assert reasons == ["scale_exceeds_max"] * 3


class TestUnprocessable422:
    def test_position_discontinuity_stays_422(self):
        payload = base_payload()
        payload["segments"][1]["control_positions"]["j1"][0] = "1.0001"
        resp = client.post(RETRIME, json=payload)
        assert resp.status_code == 422
        assert any(e["type"] == "continuity.position" for e in resp.json()["detail"])

    def test_velocity_discontinuity_stays_422(self):
        payload = base_payload()
        payload["segments"][1]["control_positions"]["j1"][1] = "1.4001"
        resp = client.post(RETRIME, json=payload)
        assert resp.status_code == 422
        assert any(e["type"] == "continuity.velocity" for e in resp.json()["detail"])

    def test_non_positive_cycle_duration(self):
        for bad in ("0", "0.0", "-0.1", "-1E-3"):
            resp = client.post(RETRIME, json=base_payload(cycle=bad))
            assert resp.status_code == 422, bad
            assert resp.json()["detail"][0]["loc"] == ["body", "cycle_duration"]

    def test_invalid_cycle_notation(self):
        for bad in ("abc", "1/4", "NaN", "Infinity", ""):
            resp = client.post(RETRIME, json=base_payload(cycle=bad))
            assert resp.status_code == 422, bad

    def test_max_scale_range(self):
        for bad in (0, -1, 1_000_001):
            resp = client.post(RETRIME, json=base_payload(max_scale=bad))
            assert resp.status_code == 422, bad
            assert resp.json()["detail"][0]["loc"] == ["body", "max_scale"]

    def test_max_scale_must_be_integer(self):
        for bad in (1.5, "5", 5.0, True, None):
            resp = client.post(RETRIME, json=base_payload(max_scale=bad))
            assert resp.status_code == 422, bad

    def test_missing_fields(self):
        payload = base_payload()
        del payload["cycle_duration"]
        assert client.post(RETRIME, json=payload).status_code == 422
        payload = base_payload()
        del payload["max_scale"]
        assert client.post(RETRIME, json=payload).status_code == 422

    def test_extra_fields_rejected(self):
        payload = base_payload()
        payload["unexpected"] = 1
        assert client.post(RETRIME, json=payload).status_code == 422

    def test_non_positive_segment_duration_stays_422(self):
        payload = base_payload()
        payload["segments"][0]["duration"] = "0"
        assert client.post(RETRIME, json=payload).status_code == 422

    def test_structure_errors_stay_422(self):
        payload = base_payload()
        payload["segments"][0]["control_positions"]["j2"] = ["0", "0", "0", "0"]
        assert client.post(RETRIME, json=payload).status_code == 422


class TestAuditContractUntouched:
    def test_audit_still_behaves_as_before(self):
        payload = {
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
        resp = client.post("/api/trajectories/audit", json=payload)
        assert resp.status_code == 200
        assert resp.json() == {
            "approved": True,
            "joints": [
                {"joint_index": 0, "joint": "j1",
                 "peak_velocity": "2.4", "peak_acceleration": "4.8"}
            ],
            "violations": [],
        }

    def test_audit_rejects_retime_only_fields_if_sent(self):
        payload = {
            "joints": [
                {"id": "j1", "travel": {"min": "-2", "max": "2"},
                 "velocity_limit": "3", "acceleration_limit": "20"}
            ],
            "segments": [
                {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}}
            ],
            "cycle_duration": "0.1",
            "max_scale": 10,
        }
        assert client.post("/api/trajectories/audit", json=payload).status_code == 422
