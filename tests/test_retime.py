"""Unit tests for exact whole-cycle retiming mathematics."""

from fractions import Fraction

import pytest

from app.retime import _ceil_div, _ceil_sqrt_fraction
from app.schemas import RetimeRequest
from app.retime import run_retime, RetimeConflictError

F = Fraction


def request(joints, segments, cycle, max_scale=1_000_000):
    return RetimeRequest.model_validate(
        {"joints": joints, "segments": segments,
         "cycle_duration": cycle, "max_scale": max_scale}
    )


JOINT = lambda v="100", a="1000", lo="-10", hi="10": {
    "id": "j1",
    "travel": {"min": lo, "max": hi},
    "velocity_limit": v,
    "acceleration_limit": a,
}


class TestCeilHelpers:
    @pytest.mark.parametrize(
        "p,q,expected",
        [(1, 1, 1), (3, 2, 2), (4, 2, 2), (24, 10, 3), (1, 100, 1), (100, 1, 100)],
    )
    def test_ceil_div(self, p, q, expected):
        assert _ceil_div(p, q) == expected

    @pytest.mark.parametrize(
        "value,expected",
        [
            (F(1), 1),
            (F(1, 4), 1),
            (F(9, 4), 2),    # sqrt = 1.5
            (F(24, 10), 2),  # sqrt(2.4) ~ 1.549
            (F(4), 2),       # exact boundary: n=2
            (F(5), 3),
            (F(100, 1), 10),
            (F(101, 100), 2),  # sqrt(1.01) just over 1
        ],
    )
    def test_ceil_sqrt_fraction(self, value, expected):
        n = _ceil_sqrt_fraction(value)
        assert n == expected
        assert n * n >= value
        assert (n - 1) * (n - 1) < value


class TestPeriodAlignment:
    def test_already_aligned_scale_one(self):
        # Two C0+C1 continuous segments, durations 0.5 each, cycle 0.1.
        joints = [JOINT()]
        segments = [
            {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
            {"duration": "0.5", "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"]}},
        ]
        result = run_retime(request(joints, segments, "0.1"))
        assert result.scale == 1
        assert [(s.duration, s.cycles) for s in result.segments] == [("0.5", 5), ("0.5", 5)]

    def test_lcm_across_segments(self):
        # Durations 0.5 (= 5/2 cycles of 0.2) and 0.3 (= 3/2): both need an
        # even scale, so the minimal scale is 2.
        joints = [JOINT()]
        segments = [
            {"duration": "0.5", "control_positions": {"j1": ["0", "0", "0.1", "0.2"]}},
            {"duration": "0.3", "control_positions": {"j1": ["0.2", "0.26", "0.3", "0.3"]}},
        ]
        result = run_retime(request(joints, segments, "0.2"))
        assert result.scale == 2
        assert [(s.duration, s.cycles) for s in result.segments] == [
            ("1", 5),
            ("0.6", 3),
        ]

    def test_nonterminating_ratio_uses_exact_fraction(self):
        # 0.5 / 0.3 = 5/3 exactly in decimal rationals: scale must be 3 and
        # the new duration is exactly 5 cycles (1.5), never 4.9999...
        joints = [JOINT()]
        segments = [
            {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
            {"duration": "0.5", "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"]}},
        ]
        result = run_retime(request(joints, segments, "0.3"))
        assert result.scale == 3
        assert all(s.cycles == 5 for s in result.segments)
        assert all(s.duration == "1.5" for s in result.segments)

    def test_scale_must_be_a_multiple_of_period_lcm(self):
        # Period: T/C = 0.5/0.3 = 5/3 -> the scale must be a *multiple* of 3.
        # Limits: peak velocity 2.4 with limit 0.5 needs n >= ceil(4.8) = 5;
        # peak acceleration 4.8 with limit 0.2 needs n >= ceil(sqrt(24)) = 5.
        # n=5 satisfies the limits but breaks period alignment, so the
        # answer is the smallest multiple of 3 not below 5, namely 6.
        joints = [JOINT(v="0.5", a="0.2")]
        segments = [
            {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
            {"duration": "0.5", "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"]}},
        ]
        result = run_retime(request(joints, segments, "0.3"))
        assert result.scale == 6
        assert [(s.duration, s.cycles) for s in result.segments] == [
            ("3", 10),
            ("3", 10),
        ]
        assert result.joints[0].peak_velocity == "0.4"
        assert result.joints[0].peak_acceleration == "2/15"
        # max_scale 5 must be reported infeasible even though the limits
        # alone would accept it.
        with pytest.raises(RetimeConflictError) as exc:
            run_retime(request(joints, segments, "0.3", max_scale=5))
        assert exc.value.reason == "scale_exceeds_max"

    def test_distinct_segment_denominators(self):
        # 0.5 / 0.2 = 5/2 (denominator 2), 0.25 / 0.2 = 5/4 (denominator 4)
        # -> lcm 4.
        joints = [JOINT(v="100")]
        segments = [
            {"duration": "0.5", "control_positions": {"j1": ["0", "0", "0", "1"]}},
            {"duration": "0.25", "control_positions": {"j1": ["1", "1.5", "2", "2"]}},
        ]
        result = run_retime(request(joints, segments, "0.2"))
        assert result.scale == 4
        assert [(s.duration, s.cycles) for s in result.segments] == [
            ("2", 10),
            ("1", 5),
        ]

    def test_cycle_longer_than_segment_still_positive_cycles(self):
        # T=0.1 < C=0.3: ratio 1/3 -> scale 3, new duration exactly one cycle.
        joints = [JOINT()]
        segments = [
            {"duration": "0.1", "control_positions": {"j1": ["0", "0.04", "0.12", "0.2"]}},
            {"duration": "0.1", "control_positions": {"j1": ["0.2", "0.28", "0.32", "0.32"]}},
        ]
        result = run_retime(request(joints, segments, "0.3"))
        assert result.scale == 3
        assert [(s.duration, s.cycles) for s in result.segments] == [
            ("0.3", 1),
            ("0.3", 1),
        ]


class TestLimitDrivenScales:
    def _two_seg_payload(self, v, a):
        # Geometry: original peak velocity 2.4, peak acceleration 4.8.
        joints = [JOINT(v=v, a=a)]
        segments = [
            {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
            {"duration": "0.5", "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"]}},
        ]
        return joints, segments

    def test_velocity_drives_scale(self):
        # ceil(2.4 / 1.5) = 2; acceleration 4.8/4 = 1.2 then also fits.
        joints, segments = self._two_seg_payload(v="1.5", a="20")
        result = run_retime(request(joints, segments, "0.5"))
        assert result.scale == 2
        assert [s.duration for s in result.segments] == ["1", "1"]
        peak = result.joints[0]
        assert peak.peak_velocity == "1.2"
        assert peak.peak_acceleration == "1.2"

    def test_acceleration_drives_scale(self):
        # n**2 >= 4.8/2 = 2.4 -> n = 2; velocity at n=2 is 1.2 <= 3.
        joints, segments = self._two_seg_payload(v="3", a="2")
        result = run_retime(request(joints, segments, "0.5"))
        assert result.scale == 2
        assert result.joints[0].peak_acceleration == "1.2"

    def test_velocity_exactly_equal_to_limit_passes_at_boundary(self):
        # 2.4 / 2.4 = 1 exactly -> scale 1 already legal (closed bounds).
        joints, segments = self._two_seg_payload(v="2.4", a="4.8")
        result = run_retime(request(joints, segments, "0.5"))
        assert result.scale == 1
        assert result.joints[0].peak_velocity == "2.4"
        assert result.joints[0].peak_acceleration == "4.8"

    def test_retimed_peak_lands_exactly_on_limit(self):
        # Velocity: n >= 2.4/1.2 = 2. Acceleration: n**2 >= 4.8/1.2 = 4 -> 2.
        joints, segments = self._two_seg_payload(v="1.2", a="1.2")
        result = run_retime(request(joints, segments, "0.5"))
        assert result.scale == 2
        assert result.joints[0].peak_velocity == "1.2"
        assert result.joints[0].peak_acceleration == "1.2"

    def test_interior_velocity_extremum_drives_scale(self):
        # [0,0,1,1] over T=1: interior velocity peak 3/2, accel peak 6.
        joints = [JOINT(v="1", a="10")]
        segments = [{"duration": "1", "control_positions": {"j1": ["0", "0", "1", "1"]}}]
        result = run_retime(request(joints, segments, "1"))
        # ceil(1.5 / 1) = 2; accel 6/4 = 1.5 <= 10.
        assert result.scale == 2
        assert result.joints[0].peak_velocity == "0.75"
        assert result.joints[0].peak_acceleration == "1.5"

    def test_nonterminating_peak_rationals(self):
        # v peak 1/6, a peak 20/27 over T=0.9 (see audit unit tests).
        joints = [JOINT(v="0.01", a="10", lo="-1", hi="1")]  # velocity limit 1/100
        segments = [{"duration": "0.9", "control_positions": {"j1": ["0", "0", "0.1", "0.1"]}}]
        result = run_retime(request(joints, segments, "0.3"))
        # period: 0.9/0.3 = 3 -> scale 1 for the period.
        # velocity: n >= ceil((1/6)/(1/100)) = ceil(100/6) = 17.
        assert result.scale == 17
        assert result.segments[0].duration == "15.3"
        assert result.segments[0].cycles == 51
        # retimed velocity peak = (1/6)/17 = 1/102 <= 1/100.
        assert result.joints[0].peak_velocity == "1/102"

    def test_multi_joint_takes_largest_requirement(self):
        joints = [
            {"id": "j1", "travel": {"min": "-10", "max": "10"},
             "velocity_limit": "1.2", "acceleration_limit": "1000"},  # needs 2
            {"id": "j2", "travel": {"min": "-10", "max": "10"},
             "velocity_limit": "100", "acceleration_limit": "1.2"},   # needs 2 (4.8/4=1.2)
        ]
        segments = [
            {"duration": "0.5",
             "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"],
                                    "j2": ["0", "0.2", "0.6", "1.0"]}},
            {"duration": "0.5",
             "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"],
                                    "j2": ["1.0", "1.4", "1.6", "1.6"]}},
        ]
        result = run_retime(request(joints, segments, "0.5"))
        assert result.scale == 2

    def test_unified_scale_must_satisfy_every_segment(self):
        # Three C0+C1 continuous segments, T=0.3 each:
        # seg0 constant velocity 1, seg1 ramps velocity 1 -> 3 (peak 3),
        # seg2 constant velocity 3. The schedule must slow ALL segments.
        joints = [JOINT(v="1", a="1000")]
        segments = [
            {"duration": "0.3", "control_positions": {"j1": ["0", "0.1", "0.2", "0.3"]}},
            {"duration": "0.3", "control_positions": {"j1": ["0.3", "0.4", "0.7", "1.0"]}},
            {"duration": "0.3", "control_positions": {"j1": ["1.0", "1.3", "1.6", "1.9"]}},
        ]
        result = run_retime(request(joints, segments, "0.1"))
        assert result.scale == 3
        assert [t.duration for t in result.segments] == ["0.9", "0.9", "0.9"]
        assert result.joints[0].peak_velocity == "1"  # 3/3 exactly on the limit


class TestZeroLimits:
    def test_stationary_joint_with_zero_limits_succeeds(self):
        joints = [JOINT(v="0", a="0")]
        segments = [
            {"duration": "0.5", "control_positions": {"j1": ["1.5", "1.5", "1.5", "1.5"]}},
            {"duration": "0.5", "control_positions": {"j1": ["1.5", "1.5", "1.5", "1.5"]}},
        ]
        result = run_retime(request(joints, segments, "0.1"))
        assert result.scale == 1
        assert result.joints[0].peak_velocity == "0"
        assert result.joints[0].peak_acceleration == "0"

    def test_constant_velocity_with_zero_acceleration_limit_succeeds(self):
        joints = [JOINT(v="3", a="0")]
        segments = [{"duration": "1", "control_positions": {"j1": ["0", "1", "2", "3"]}}]
        result = run_retime(request(joints, segments, "1"))
        assert result.scale == 1
        assert result.joints[0].peak_acceleration == "0"
        assert result.joints[0].peak_velocity == "3"

    def test_zero_velocity_limit_with_motion_conflicts(self):
        joints = [JOINT(v="0", a="20")]
        segments = [{"duration": "1", "control_positions": {"j1": ["0", "1", "2", "3"]}}]
        with pytest.raises(RetimeConflictError) as exc:
            run_retime(request(joints, segments, "1"))
        assert exc.value.reason == "zero_limit_with_motion"

    def test_zero_acceleration_limit_with_curvature_conflicts(self):
        joints = [JOINT(v="10", a="0")]
        segments = [{"duration": "1", "control_positions": {"j1": ["0", "0", "1", "1"]}}]
        with pytest.raises(RetimeConflictError) as exc:
            run_retime(request(joints, segments, "1"))
        assert exc.value.reason == "zero_limit_with_motion"


class TestConflicts:
    def test_travel_out_of_bounds_conflicts(self):
        joints = [JOINT(lo="-0.5", hi="0.5", v="100", a="1000")]
        segments = [{"duration": "1", "control_positions": {"j1": ["0", "1", "1", "0"]}}]
        with pytest.raises(RetimeConflictError) as exc:
            run_retime(request(joints, segments, "1"))
        assert exc.value.reason == "travel_out_of_bounds"

    def test_travel_precedence_over_zero_limit(self):
        joints = [{"id": "j1", "travel": {"min": "-0.5", "max": "0.5"},
                   "velocity_limit": "0", "acceleration_limit": "0"}]
        segments = [{"duration": "1", "control_positions": {"j1": ["0", "1", "1", "0"]}}]
        with pytest.raises(RetimeConflictError) as exc:
            run_retime(request(joints, segments, "1"))
        assert exc.value.reason == "travel_out_of_bounds"

    def test_zero_limit_precedence_over_scale(self):
        # Motion exists (zero-limit conflict) and the needed scale would also
        # be huge; the zero-limit reason must win deterministically.
        joints = [{"id": "j1", "travel": {"min": "-10", "max": "10"},
                   "velocity_limit": "0", "acceleration_limit": "1"}]
        segments = [{"duration": "1", "control_positions": {"j1": ["0", "0", "1", "1"]}}]
        with pytest.raises(RetimeConflictError) as exc:
            run_retime(request(joints, segments, "1", max_scale=1))
        assert exc.value.reason == "zero_limit_with_motion"

    def test_required_scale_above_max_conflicts(self):
        joints = [JOINT(v="1", a="10")]
        segments = [{"duration": "1", "control_positions": {"j1": ["0", "0", "1", "1"]}}]
        with pytest.raises(RetimeConflictError) as exc:
            run_retime(request(joints, segments, "1", max_scale=1))
        assert exc.value.reason == "scale_exceeds_max"

    def test_required_scale_equal_to_max_is_accepted(self):
        joints = [JOINT(v="1", a="10")]
        segments = [{"duration": "1", "control_positions": {"j1": ["0", "0", "1", "1"]}}]
        result = run_retime(request(joints, segments, "1", max_scale=2))
        assert result.scale == 2

    def test_period_alone_can_exceed_max(self):
        # 0.5/0.3 = 5/3 needs scale >= 3 regardless of limits.
        joints = [JOINT(v="1000", a="1000")]
        segments = [
            {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
            {"duration": "0.5", "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"]}},
        ]
        with pytest.raises(RetimeConflictError) as exc:
            run_retime(request(joints, segments, "0.3", max_scale=2))
        assert exc.value.reason == "scale_exceeds_max"

    def test_max_scale_upper_bound_accepted(self):
        joints = [JOINT()]
        segments = [{"duration": "1", "control_positions": {"j1": ["0", "0", "0", "0"]}}]
        result = run_retime(request(joints, segments, "1", max_scale=1_000_000))
        assert result.scale == 1


class TestMinimalityProof:
    def test_no_smaller_scale_works(self):
        """Independently exhaust every integer 1..scale-1.

        For each candidate we recompute, straight from the Bézier endpoint/
        interior-extremum formulas: (a) whole-cycle alignment and (b) the
        exact velocity/acceleration bounds. Every smaller candidate must fail
        something; the chosen scale satisfies everything.
        """
        joints = [
            {"id": "j1", "travel": {"min": "-10", "max": "10"},
             "velocity_limit": "1", "acceleration_limit": "2"},
            {"id": "j2", "travel": {"min": "-10", "max": "10"},
             "velocity_limit": "10", "acceleration_limit": "0.4"},
        ]
        segments = [
            {"duration": "0.5",
             "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"],
                                    "j2": ["0", "0.2", "0.6", "1.0"]}},
            {"duration": "0.3",
             # starts at 1.0 with velocity 3*0.24/0.3 = 2.4, matching the
             # 3*0.4/0.5 = 2.4 the previous segment ends with.
             "control_positions": {"j1": ["1.0", "1.24", "1.4", "1.4"],
                                    "j2": ["1.0", "1.24", "1.4", "1.4"]}},
        ]
        result = run_retime(request(joints, segments, "0.2"))
        scale = result.scale
        assert scale > 1

        # Independent recomputation of geometry peaks at original timing.
        from app.audit import segment_peaks

        durations = [F(5, 10), F(3, 10)]
        cycle = F(2, 10)
        peaks = []
        for s, segment in enumerate(segments):
            row = {}
            for j, joint in enumerate(joints):
                cps = [F(x) for x in segment["control_positions"][joint["id"]]]
                row[joint["id"]] = segment_peaks(cps, durations[s])
            peaks.append(row)

        def feasible(n):
            for s, duration in enumerate(durations):
                if (n * duration / cycle).denominator != 1:
                    return False
            for j, joint in enumerate(joints):
                vlim = F(joint["velocity_limit"])
                alim = F(joint["acceleration_limit"])
                for s in range(len(segments)):
                    pv, pa = peaks[s][joint["id"]]
                    if pv / n > vlim or pa / (n * n) > alim:
                        return False
            return True

        for n in range(1, scale):
            assert not feasible(n), f"scale {n} unexpectedly feasible (< {scale})"
        assert feasible(scale)
        # And every returned duration is an exact positive whole cycle count.
        for s, timing in enumerate(result.segments):
            assert timing.cycles > 0
            assert F(timing.duration) == timing.cycles * cycle


class TestDecimalEquivalence:
    def test_equivalent_cycle_spellings_identical(self):
        joints = [JOINT()]
        segments = [
            {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
            {"duration": "0.5", "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"]}},
        ]
        bodies = [
            run_retime(request(joints, segments, cycle)).model_dump()
            for cycle in ("0.1", "0.10", "1E-1", "100E-3", "+0.1")
        ]
        assert all(body == bodies[0] for body in bodies[1:])

    def test_equivalent_duration_spellings_identical(self):
        def build(d0, d1):
            joints = [JOINT(v="1", a="10")]
            segments = [
                {"duration": d0, "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
                {"duration": d1, "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"]}},
            ]
            return run_retime(request(joints, segments, "0.3")).model_dump()

        a = build("0.5", "0.5")
        b = build("0.50", "500E-3")
        c = build("5E-1", ".5")
        assert a == b == c
        assert a["scale"] == 3
