"""Exact whole-cycle retiming of piecewise cubic Bézier trajectories.

The controller only accepts segment durations that are whole multiples of
its control period. A *retime* keeps the path geometry (the Bézier control
points) identical and multiplies every segment duration by one positive
integer ``scale``:

* new duration of segment ``s``:   ``d'_s = scale * T_s``;
* its velocity scales by ``1/scale``   (a degree-2 Bézier of the path);
* its acceleration scales by ``1/scale**2`` (a degree-1 Bézier).

Multiplying all durations by the *same* integer preserves the path shape
and both C0 and C1 continuity: an endpoint velocity ``3*(c3-c2)/T_s``
becomes ``... / (scale*T_s)`` on both sides of every junction.

All reasoning is exact rational arithmetic — no float tolerance, no
sampling. For each segment the peak |velocity| ``V_s`` and |acceleration|
``A_s`` at original timing come from the same analytical adjudication as
/audit (``segment_peaks``). A scale ``n`` is feasible iff:

1. whole-cycle durations: ``n*T_s / C`` is a positive integer for every
   segment ``s`` with control period ``C``. Writing ``T_s/C = p_s/q_s`` in
   lowest terms this means ``q_s | n`` for every ``s`` (as
   ``gcd(p_s, q_s) = 1``), hence ``n`` must be a multiple of
   ``L = lcm_s(q_s)``;
2. velocity:  ``V_s/n   <= vlim_j``, i.e. ``n >= ceil(V_s/vlim_j)``,
   for every positive velocity limit;
3. acceleration: ``A_s/n**2 <= alim_j``, i.e.
   ``n >= ceil(sqrt(A_s/alim_j))``, for every positive acceleration limit.

Let ``B`` be the largest of the velocity/acceleration integer lower
bounds. The smallest feasible scale is the smallest *multiple of L* that
is not below ``B`` — namely ``L * ceil(B/L)``. Every smaller positive
integer either breaks the period divisibility or one of the limits, so
the returned scale is provably minimal. A joint whose limit is exactly
zero is only feasible when the corresponding peak is also exactly zero
(slowing down can never remove genuine motion).
"""

from __future__ import annotations

import math
from fractions import Fraction
from typing import Dict, List, Tuple

from .audit import format_exact, prepare_trajectory, segment_peaks, travel_violations
from .schemas import (
    JointPeak,
    RetimeRequest,
    RetimeResponse,
    SegmentTiming,
)

# Stable machine-readable 409 reason codes.
REASON_TRAVEL = "travel_out_of_bounds"
REASON_ZERO_LIMIT = "zero_limit_with_motion"
REASON_SCALE = "scale_exceeds_max"


class RetimeConflictError(Exception):
    """A well-formed, structurally valid trajectory that cannot be scheduled.

    Maps to HTTP 409 with a stable ``reason`` code.
    """

    def __init__(self, reason: str, msg: str):
        super().__init__(msg)
        self.reason = reason
        self.msg = msg


def _ceil_div(numerator: int, denominator: int) -> int:
    """ceil(numerator / denominator) for positive integers, exactly."""
    return (numerator + denominator - 1) // denominator


def _ceil_sqrt_fraction(value: Fraction) -> int:
    """Smallest positive integer ``n`` with ``n**2 >= value`` (value > 0)."""
    # ceil(sqrt(p/q)) == floor(sqrt((p-1)/q)) + 1, all in integers.
    return math.isqrt((value.numerator - 1) // value.denominator) + 1


def _segment_peaks(
    req: RetimeRequest,
    durations: List[Fraction],
    positions: List[Dict[str, List[Fraction]]],
) -> List[List[Tuple[Fraction, Fraction]]]:
    """Exact original-timing (peak |velocity|, peak |acceleration|) per seg/joint."""
    peaks: List[List[Tuple[Fraction, Fraction]]] = []
    for seg_index in range(len(req.segments)):
        row: List[Tuple[Fraction, Fraction]] = []
        duration = durations[seg_index]
        for joint in req.joints:
            row.append(segment_peaks(positions[seg_index][joint.id], duration))
        peaks.append(row)
    return peaks


def run_retime(req: RetimeRequest) -> RetimeResponse:
    """Compute the minimal whole-cycle scale, or raise a 409 conflict."""
    durations, positions, limits = prepare_trajectory(req)
    cycle = Fraction(req.cycle_duration)
    max_scale = int(req.max_scale)

    peaks = _segment_peaks(req, durations, positions)

    # --- Travel is timing-independent (the path shape never changes). -------
    for seg_index in range(len(req.segments)):
        for joint_index, joint in enumerate(req.joints):
            low, high, _v_limit, _a_limit = limits[joint_index]
            crossed = travel_violations(positions[seg_index][joint.id], low, high)
            if crossed:
                bound, value, point_index = crossed[0]
                bound_value = low if bound == "lower" else high
                raise RetimeConflictError(
                    REASON_TRAVEL,
                    (
                        f"joint {joint.id!r} control point {point_index} of segment "
                        f"{seg_index} is {format_exact(value)}, outside the closed "
                        f"travel interval [{format_exact(low)}, {format_exact(high)}] "
                        f"(crossed the {bound} bound {format_exact(bound_value)}); "
                        "retiming preserves the path shape and cannot repair this"
                    ),
                )

    # --- Zero limits with nonzero motion can never be satisfied. -----------
    # Deterministic first offender: segment, then joint, velocity before
    # acceleration (the audit constraint order).
    for seg_index in range(len(req.segments)):
        for joint_index, joint in enumerate(req.joints):
            v_peak, a_peak = peaks[seg_index][joint_index]
            _low, _high, v_limit, a_limit = limits[joint_index]
            if v_limit == 0 and v_peak != 0:
                raise RetimeConflictError(
                    REASON_ZERO_LIMIT,
                    (
                        f"joint {joint.id!r} has velocity_limit 0 but segment "
                        f"{seg_index} has nonzero peak velocity "
                        f"{format_exact(v_peak)} at any finite duration; a zero "
                        "velocity limit requires a stationary joint"
                    ),
                )
            if a_limit == 0 and a_peak != 0:
                raise RetimeConflictError(
                    REASON_ZERO_LIMIT,
                    (
                        f"joint {joint.id!r} has acceleration_limit 0 but segment "
                        f"{seg_index} has nonzero peak acceleration "
                        f"{format_exact(a_peak)} at any finite duration; a zero "
                        "acceleration limit requires a piecewise-linear joint"
                    ),
                )

    # --- Exact integer lower bounds on the unified scale. ------------------
    # (1) Whole-cycle durations: the scale must be a *multiple* of L, the
    # lcm of the reduced denominators of T_s / C.
    period_lcm = 1
    for duration in durations:
        ratio = duration / cycle
        if ratio.denominator > 1:
            period_lcm = math.lcm(period_lcm, ratio.denominator)

    # (2/3) Velocity/acceleration only demand a plain integer lower bound B.
    bound = 1
    for seg_index in range(len(req.segments)):
        for joint_index, joint in enumerate(req.joints):
            v_peak, a_peak = peaks[seg_index][joint_index]
            _low, _high, v_limit, a_limit = limits[joint_index]
            if v_limit > 0 and v_peak > 0:
                needed = _ceil_div(v_peak.numerator * v_limit.denominator,
                                   v_peak.denominator * v_limit.numerator)
                if needed > bound:
                    bound = needed
            if a_limit > 0 and a_peak > 0:
                # smallest n with n**2 >= a_peak / a_limit
                needed = _ceil_sqrt_fraction(a_peak / a_limit)
                if needed > bound:
                    bound = needed

    # Smallest multiple of period_lcm not below the limit bound: at this
    # scale every n*T_s/C is integral and both derivative families fit.
    required = period_lcm * _ceil_div(bound, period_lcm)

    if required > max_scale:
        raise RetimeConflictError(
            REASON_SCALE,
            (
                f"the minimal unified scale satisfying whole-cycle durations and "
                f"all velocity/acceleration limits is {required}, which exceeds "
                f"max_scale {max_scale}"
            ),
        )

    scale = required

    # --- Build the exact retimed schedule and retimed peaks. ---------------
    segment_timings: List[SegmentTiming] = []
    peak_velocity = [Fraction(0)] * len(req.joints)
    peak_acceleration = [Fraction(0)] * len(req.joints)
    for seg_index, duration in enumerate(durations):
        new_duration = scale * duration
        cycles = new_duration / cycle
        # Feasibility construction guarantees an exact whole number of cycles.
        assert cycles.denominator == 1 and cycles.numerator > 0
        segment_timings.append(
            SegmentTiming(
                segment_index=seg_index,
                duration=format_exact(new_duration),
                cycles=cycles.numerator,
            )
        )
        for joint_index in range(len(req.joints)):
            v_peak, a_peak = peaks[seg_index][joint_index]
            retimed_v = v_peak / scale
            retimed_a = a_peak / (scale * scale)
            if retimed_v > peak_velocity[joint_index]:
                peak_velocity[joint_index] = retimed_v
            if retimed_a > peak_acceleration[joint_index]:
                peak_acceleration[joint_index] = retimed_a

    return RetimeResponse(
        scale=scale,
        cycle_duration=format_exact(cycle),
        segments=segment_timings,
        joints=[
            JointPeak(
                joint_index=i,
                joint=joint.id,
                peak_velocity=format_exact(peak_velocity[i]),
                peak_acceleration=format_exact(peak_acceleration[i]),
            )
            for i, joint in enumerate(req.joints)
        ],
    )
