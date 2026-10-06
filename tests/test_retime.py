"""API and unit tests for POST /api/trajectories/retime.

All expectations below are computed by exact rational arithmetic: a uniform
integer scale ``k`` turns segment durations ``T_i`` into ``k * T_i``, velocity
peaks into ``peak_v / k`` and acceleration peaks into ``peak_a / k**2``.
"""

from fractions import Fraction

from fastapi.testclient import TestClient

from app.main import app
from app.retime import ceil_sqrt_ratio, cycle_multiple, limit_min_scale

client = TestClient(app)
RETIME = "/api/trajectories/retime"
AUDIT = "/api/trajectories/audit"
F = Fraction


def base_payload():
    """Two C0+C1-continuous segments; exact peaks pv=2.4, pa=4.8 for j1."""
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
        "cycle_duration": "0.2",
        "max_scale": 1000,
    }


def post(payload):
    return client.post(RETIME, json=payload)


class TestHelpers:
    def test_ceil_sqrt_ratio_exact(self):
        assert ceil_sqrt_ratio(F(0)) == 0
        assert ceil_sqrt_ratio(F(1)) == 1
        assert ceil_sqrt_ratio(F(2)) == 2
        assert ceil_sqrt_ratio(F(4)) == 2
        assert ceil_sqrt_ratio(F(9, 2)) == 3
        assert ceil_sqrt_ratio(F(1, 4)) == 1
        assert ceil_sqrt_ratio(F(10**24)) == 10**12
        assert ceil_sqrt_ratio(F(10**24 + 1)) == 10**12 + 1
        value = F(123456789123456789, 987654321)
        k = ceil_sqrt_ratio(value)
        assert (k - 1) ** 2 < value <= k * k

    def test_cycle_multiple_is_lcm_of_ratio_denominators(self):
        assert cycle_multiple([F(1, 2)], F(1, 5)) == 2  # 0.5/0.2 = 5/2
        assert cycle_multiple([F(1, 2)], F(1, 4)) == 1  # already integral
        assert cycle_multiple([F(3, 10)], F(7, 10)) == 7
        assert cycle_multiple([F(1, 2), F(1, 3), F(1, 5)], F(1)) == 30

    def test_limit_min_scale_boundary_equality(self):
        # pv=2.4 with limit 1.2 needs k >= 2 exactly (equality qualifies).
        assert limit_min_scale([(F(12, 5), F(0))], [F(6, 5)], [F(0)]) == 2
        # pa=4 with limit 1 needs k >= 2 exactly; pa=4.8 needs k >= 3.
        assert limit_min_scale([(F(0), F(4))], [F(0)], [F(1)]) == 2
        assert limit_min_scale([(F(0), F(24, 5))], [F(0)], [F(1)]) == 3
        # Zero peaks constrain nothing, even with zero limits.
        assert limit_min_scale([(F(0), F(0))], [F(0)], [F(0)]) == 1


class TestRetimeApproval:
    def test_cycle_driven_scale(self):
        # 0.5/0.2 = 5/2 -> every scale must be even; limits already fine.
        body = post(base_payload()).json()
        assert body["scale"] == 2
        assert body["cycle_duration"] == "0.2"
        assert [s["duration"] for s in body["segments"]] == ["1", "1"]
        assert [s["cycles"] for s in body["segments"]] == [5, 5]
        assert body["total_duration"] == "2"
        assert body["total_cycles"] == 10
        assert body["joints"] == [
            {
                "joint_index": 0,
                "joint": "j1",
                "peak_velocity": "1.2",
                "peak_acceleration": "1.2",
            }
        ]
        assert body["proof"] == {"cycle_multiple": 2, "limit_min_scale": 1, "minimal": True}

    def test_limit_driven_scale(self):
        # Durations already on-cycle (0.5/0.5 = 1); velocity limit forces k=3.
        payload = base_payload()
        payload["cycle_duration"] = "0.5"
        payload["joints"][0]["velocity_limit"] = "1"
        body = post(payload).json()
        assert body["scale"] == 3
        assert [s["duration"] for s in body["segments"]] == ["1.5", "1.5"]
        assert [s["cycles"] for s in body["segments"]] == [3, 3]
        assert body["joints"][0]["peak_velocity"] == "0.8"
        assert body["joints"][0]["peak_acceleration"] == "8/15"
        assert body["proof"] == {"cycle_multiple": 1, "limit_min_scale": 3, "minimal": True}

    def test_combined_cycle_and_limit(self):
        # Even scales only; velocity needs k >= 3 -> smallest even is 4.
        payload = base_payload()
        payload["joints"][0]["velocity_limit"] = "1"
        body = post(payload).json()
        assert body["scale"] == 4
        assert [s["cycles"] for s in body["segments"]] == [10, 10]
        assert body["joints"][0]["peak_velocity"] == "0.6"
        assert body["joints"][0]["peak_acceleration"] == "0.3"
        assert body["proof"]["cycle_multiple"] == 2
        assert body["proof"]["limit_min_scale"] == 3

    def test_acceleration_driven_scale(self):
        payload = base_payload()
        payload["cycle_duration"] = "0.5"
        payload["joints"][0]["acceleration_limit"] = "1"  # k^2 >= 4.8 -> k >= 3
        body = post(payload).json()
        assert body["scale"] == 3
        assert body["joints"][0]["peak_acceleration"] == "8/15"

    def test_scale_one_when_already_compliant(self):
        payload = base_payload()
        payload["cycle_duration"] = "0.5"  # durations already whole cycles
        body = post(payload).json()
        assert body["scale"] == 1
        assert [s["duration"] for s in body["segments"]] == ["0.5", "0.5"]
        assert [s["cycles"] for s in body["segments"]] == [1, 1]
        assert body["joints"][0]["peak_velocity"] == "2.4"
        assert body["joints"][0]["peak_acceleration"] == "4.8"

    def test_static_trajectory_with_zero_limits(self):
        # Zero peaks satisfy zero limits; only the cycle grid matters.
        payload = base_payload()
        payload["joints"][0]["velocity_limit"] = "0"
        payload["joints"][0]["acceleration_limit"] = "0"
        for seg in payload["segments"]:
            seg["control_positions"]["j1"] = ["0.5", "0.5", "0.5", "0.5"]
        body = post(payload).json()
        assert body["scale"] == 2  # 0.5/0.2 = 5/2 -> even scales only
        assert body["joints"][0]["peak_velocity"] == "0"
        assert body["joints"][0]["peak_acceleration"] == "0"

    def test_nonterminating_peaks_stay_exact(self):
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
            "cycle_duration": "0.2",  # 0.9/0.2 = 9/2 -> even scales
            "max_scale": 100,
        }
        body = post(payload).json()
        assert body["scale"] == 2
        assert body["segments"][0]["duration"] == "1.8"
        assert body["segments"][0]["cycles"] == 9
        assert body["joints"][0]["peak_velocity"] == "1/12"
        assert body["joints"][0]["peak_acceleration"] == "5/27"

    def test_mixed_durations_use_lcm(self):
        # 0.5/0.2 = 5/2 and 0.3/0.2 = 3/2 -> scales must be even (lcm 2).
        payload = {
            "joints": [
                {
                    "id": "j1",
                    "travel": {"min": "-10", "max": "10"},
                    "velocity_limit": "10",
                    "acceleration_limit": "10",
                }
            ],
            "segments": [
                {"duration": "0.5", "control_positions": {"j1": ["0", "0", "0.1", "0.2"]}},
                {"duration": "0.3", "control_positions": {"j1": ["0.2", "0.26", "0.3", "0.3"]}},
            ],
            "cycle_duration": "0.2",
            "max_scale": 100,
        }
        body = post(payload).json()
        assert body["scale"] == 2
        assert [s["duration"] for s in body["segments"]] == ["1", "0.6"]
        assert [s["cycles"] for s in body["segments"]] == [5, 3]
        assert body["joints"][0]["peak_velocity"] == "0.3"
        assert body["joints"][0]["peak_acceleration"] == "2/3"

    def test_multiple_joints_take_max_requirement(self):
        # j1 needs k >= 2 (velocity), j2 needs k >= 3 (acceleration) -> k = 3.
        payload = {
            "joints": [
                {
                    "id": "j1",
                    "travel": {"min": "-2", "max": "2"},
                    "velocity_limit": "1.2",
                    "acceleration_limit": "100",
                },
                {
                    "id": "j2",
                    "travel": {"min": "-2", "max": "2"},
                    "velocity_limit": "10",
                    "acceleration_limit": "1",
                },
            ],
            "segments": [
                {
                    "duration": "0.5",
                    "control_positions": {
                        "j1": ["0", "0.2", "0.6", "1.0"],
                        "j2": ["0", "0", "0.3", "0.3"],
                    },
                },
                {
                    "duration": "0.5",
                    "control_positions": {
                        "j1": ["1.0", "1.4", "1.6", "1.6"],
                        "j2": ["0.3", "0.3", "0.5", "0.5"],
                    },
                },
            ],
            "cycle_duration": "0.5",
            "max_scale": 100,
        }
        body = post(payload).json()
        assert body["scale"] == 3
        j1, j2 = body["joints"]
        assert (j1["peak_velocity"], j1["peak_acceleration"]) == ("0.8", "8/15")
        assert (j2["peak_velocity"], j2["peak_acceleration"]) == ("0.3", "0.8")

    def test_minimality_certificate(self):
        payload = base_payload()
        payload["joints"][0]["velocity_limit"] = "1"
        body = post(payload).json()
        scale, proof = body["scale"], body["proof"]
        multiple, k_min = proof["cycle_multiple"], proof["limit_min_scale"]
        # The certificate: feasible scales are exactly the multiples of
        # `multiple` that are >= k_min; `scale` is the least of them.
        assert scale % multiple == 0
        assert scale >= k_min
        assert scale - multiple < k_min  # the previous multiple cannot qualify
        assert proof["minimal"] is True

    def test_retimed_plan_passes_audit(self):
        # End-to-end invariant: the retimed trajectory is approvable.
        payload = base_payload()
        payload["joints"][0]["velocity_limit"] = "1"
        body = post(payload).json()
        audited = {
            "joints": payload["joints"],
            "segments": [
                {"duration": seg["duration"], "control_positions": src["control_positions"]}
                for seg, src in zip(body["segments"], payload["segments"])
            ],
        }
        audit_body = client.post(AUDIT, json=audited).json()
        assert audit_body["approved"] is True
        assert audit_body["joints"][0]["peak_velocity"] == body["joints"][0]["peak_velocity"]
        assert audit_body["joints"][0]["peak_acceleration"] == body["joints"][0]["peak_acceleration"]

    def test_durations_are_scale_times_original(self):
        payload = base_payload()
        payload["joints"][0]["velocity_limit"] = "1"
        body = post(payload).json()
        scale = body["scale"]
        cycle = F(1, 5)
        for plan, src in zip(body["segments"], payload["segments"]):
            original = F(src["duration"])
            assert F(plan["duration"]) == scale * original
            assert F(plan["cycles"]) * cycle == F(plan["duration"])
            assert plan["cycles"] >= 1


class TestBoundaryEquality:
    def test_velocity_peak_equal_to_limit_passes(self):
        payload = base_payload()
        payload["cycle_duration"] = "0.5"
        payload["joints"][0]["velocity_limit"] = "1.2"  # 2.4/2 == 1.2 exactly
        body = post(payload).json()
        assert body["scale"] == 2
        assert body["joints"][0]["peak_velocity"] == "1.2"

    def test_acceleration_peak_equal_to_limit_passes(self):
        payload = base_payload()
        payload["cycle_duration"] = "0.5"
        payload["joints"][0]["acceleration_limit"] = "1.2"  # 4.8/4 == 1.2 exactly
        body = post(payload).json()
        assert body["scale"] == 2
        assert body["joints"][0]["peak_acceleration"] == "1.2"

    def test_max_scale_equal_to_required_passes(self):
        payload = base_payload()
        payload["max_scale"] = 2  # required scale is exactly 2
        assert post(payload).status_code == 200

    def test_max_scale_one_below_required_conflicts(self):
        payload = base_payload()
        payload["max_scale"] = 1
        resp = post(payload)
        assert resp.status_code == 409
        assert resp.json()["detail"]["reason"] == "scale_exceeds_max"

    @staticmethod
    def _single_segment_payload(control_positions, duration, acceleration_limit):
        return {
            "joints": [
                {
                    "id": "j1",
                    "travel": {"min": "-1", "max": "1"},
                    "velocity_limit": "10",
                    "acceleration_limit": acceleration_limit,
                }
            ],
            "segments": [
                {"duration": duration, "control_positions": {"j1": control_positions}}
            ],
            "cycle_duration": duration,
            "max_scale": 100,
        }

    def test_acceleration_exactly_at_perfect_square_boundary(self):
        # pa = 6*0.24/0.6**2 = 4 exactly, so k=2 suffices for limit 1
        # (2**2 >= 4 — equality qualifies). A tolerance-based check must not
        # round this up to 3.
        payload = self._single_segment_payload(["0", "0", "0.24", "0.24"], "0.6", "1")
        body = post(payload).json()
        assert body["scale"] == 2
        assert body["joints"][0]["peak_acceleration"] == "1"

    def test_acceleration_one_ulp_above_boundary_needs_next_scale(self):
        # pa = 24 * 0.16666666666666667 = 4.00000000000000008 > 4, so k=2
        # fails (4 < pa) and k=3 is required. A float check would see 4.0 and
        # wrongly accept k=2.
        payload = self._single_segment_payload(
            ["0", "0", "0.16666666666666667", "0.16666666666666667"], "0.5", "1"
        )
        body = post(payload).json()
        assert body["scale"] == 3

    def test_velocity_ratio_exactly_integer(self):
        # pv = 3*0.8/1 = 2.4 with limit 1.2 -> k >= 2 exactly (not 3).
        payload = self._single_segment_payload(["0", "0.8", "0.8", "0"], "1", "1000")
        payload["joints"][0]["velocity_limit"] = "1.2"
        body = post(payload).json()
        assert body["scale"] == 2
        assert body["joints"][0]["peak_velocity"] == "1.2"


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
                "cycle_duration": "0.20",
                "max_scale": 1000,
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
                "cycle_duration": "2E-1",
                "max_scale": "1000",
            },
        ]
        bodies = []
        for variant in variants:
            resp = post(variant)
            assert resp.status_code == 200
            bodies.append(resp.json())
        assert bodies[0] == bodies[1] == bodies[2]

    def test_json_numbers_accepted(self):
        payload = base_payload()
        payload["cycle_duration"] = 0.2
        payload["max_scale"] = 1000.0
        resp = post(payload)
        assert resp.status_code == 200
        assert resp.json()["scale"] == 2


class TestConflicts:
    def test_travel_upper_bound_conflict(self):
        payload = base_payload()
        payload["segments"][0]["control_positions"]["j1"][1] = "5"
        resp = post(payload)
        assert resp.status_code == 409
        detail = resp.json()["detail"]
        assert detail["reason"] == "travel_out_of_bounds"
        (violation,) = detail["violations"]
        assert violation["constraint"] == "travel"
        assert violation["bound"] == "upper"
        assert violation["control_point_index"] == 1
        assert violation["value"] == "5"

    def test_travel_lower_bound_conflict(self):
        payload = base_payload()
        payload["segments"][1]["control_positions"]["j1"][2] = "-3"
        resp = post(payload)
        assert resp.status_code == 409
        detail = resp.json()["detail"]
        assert detail["reason"] == "travel_out_of_bounds"
        assert detail["violations"][0]["bound"] == "lower"
        assert detail["violations"][0]["value"] == "-3"

    def test_zero_velocity_limit_conflict(self):
        payload = base_payload()
        payload["joints"][0]["velocity_limit"] = "0"
        resp = post(payload)
        assert resp.status_code == 409
        detail = resp.json()["detail"]
        assert detail["reason"] == "zero_velocity_limit"
        assert detail["joints"] == [
            {"joint_index": 0, "joint": "j1", "peak_velocity": "2.4"}
        ]

    def test_zero_acceleration_limit_conflict(self):
        payload = base_payload()
        payload["joints"][0]["acceleration_limit"] = "0"
        resp = post(payload)
        assert resp.status_code == 409
        detail = resp.json()["detail"]
        assert detail["reason"] == "zero_acceleration_limit"
        assert detail["joints"] == [
            {"joint_index": 0, "joint": "j1", "peak_acceleration": "4.8"}
        ]

    def test_scale_exceeds_max_conflict(self):
        payload = base_payload()
        payload["max_scale"] = 1
        resp = post(payload)
        assert resp.status_code == 409
        detail = resp.json()["detail"]
        assert detail["reason"] == "scale_exceeds_max"
        assert detail["required_scale"] == "2"
        assert detail["max_scale"] == 1

    def test_scale_exceeds_max_reports_exact_required_scale(self):
        payload = base_payload()
        payload["cycle_duration"] = "0.5"
        payload["joints"][0]["velocity_limit"] = "0.001"  # needs k = 2400
        payload["max_scale"] = 1000
        resp = post(payload)
        assert resp.status_code == 409
        detail = resp.json()["detail"]
        assert detail["reason"] == "scale_exceeds_max"
        assert detail["required_scale"] == "2400"

    def test_conflict_check_order_is_stable(self):
        # Travel violation and zero limit together -> travel reported first.
        payload = base_payload()
        payload["segments"][0]["control_positions"]["j1"][1] = "5"
        payload["joints"][0]["velocity_limit"] = "0"
        resp = post(payload)
        assert resp.json()["detail"]["reason"] == "travel_out_of_bounds"

        # Zero limit and excessive scale together -> zero limit reported.
        payload = base_payload()
        payload["joints"][0]["velocity_limit"] = "0"
        payload["max_scale"] = 1
        resp = post(payload)
        assert resp.json()["detail"]["reason"] == "zero_velocity_limit"


class TestUnprocessable:
    def test_non_positive_cycle_duration(self):
        for bad in ("0", "0.0", "-0.5", "-1E-2"):
            payload = base_payload()
            payload["cycle_duration"] = bad
            resp = post(payload)
            assert resp.status_code == 422, bad
            assert resp.json()["detail"][0]["loc"] == ["body", "cycle_duration"]

    def test_invalid_cycle_duration_notations(self):
        for bad in ("abc", "1/2", "0x10", "NaN", "inf", "", "1..2"):
            payload = base_payload()
            payload["cycle_duration"] = bad
            assert post(payload).status_code == 422, bad

    def test_max_scale_out_of_range(self):
        for bad in (0, -1, 1000001, "0", "1000001"):
            payload = base_payload()
            payload["max_scale"] = bad
            assert post(payload).status_code == 422, bad

    def test_max_scale_must_be_an_integer(self):
        for bad in (1.5, "2.5", True, False, None, "abc"):
            payload = base_payload()
            payload["max_scale"] = bad
            assert post(payload).status_code == 422, bad

    def test_max_scale_bounds_accepted(self):
        for good in (1, 1000000, "1000000"):
            payload = base_payload()
            payload["max_scale"] = good
            payload["cycle_duration"] = "0.5"  # scale 1 suffices
            assert post(payload).status_code == 200, good

    def test_missing_retime_fields(self):
        for field in ("cycle_duration", "max_scale"):
            payload = base_payload()
            del payload[field]
            resp = post(payload)
            assert resp.status_code == 422
            assert resp.json()["detail"][0]["loc"] == ["body", field]

    def test_extra_fields_rejected(self):
        payload = base_payload()
        payload["speedup"] = 2
        assert post(payload).status_code == 422

    def test_position_discontinuity(self):
        payload = base_payload()
        payload["segments"][1]["control_positions"]["j1"][0] = "1.0001"
        resp = post(payload)
        assert resp.status_code == 422
        assert any(e["type"] == "continuity.position" for e in resp.json()["detail"])

    def test_velocity_discontinuity(self):
        payload = base_payload()
        payload["segments"][1]["control_positions"]["j1"][1] = "1.4001"
        resp = post(payload)
        assert resp.status_code == 422
        assert any(e["type"] == "continuity.velocity" for e in resp.json()["detail"])

    def test_structure_errors_still_422(self):
        payload = base_payload()
        payload["segments"][0]["control_positions"]["j2"] = ["0", "0", "0", "0"]
        resp = post(payload)
        assert resp.status_code == 422
        assert any(e["type"] == "value_error.unknown_joint" for e in resp.json()["detail"])

    def test_non_positive_segment_duration(self):
        payload = base_payload()
        payload["segments"][0]["duration"] = "0"
        assert post(payload).status_code == 422


class TestAuditUnchanged:
    def test_audit_contract_still_works(self):
        payload = base_payload()
        for field in ("cycle_duration", "max_scale"):
            del payload[field]
        body = client.post(AUDIT, json=payload).json()
        assert body["approved"] is True
        assert body["joints"][0]["peak_velocity"] == "2.4"
        assert body["joints"][0]["peak_acceleration"] == "4.8"

    def test_audit_rejects_retime_fields(self):
        # The audit contract is unchanged: unknown fields remain forbidden.
        resp = client.post(AUDIT, json=base_payload())
        assert resp.status_code == 422
