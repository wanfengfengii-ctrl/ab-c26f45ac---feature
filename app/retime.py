"""Exact uniform retiming of piecewise cubic Bézier trajectories.

The robot controller only accepts segment durations that are whole multiples
of its control cycle. Slowing the *entire* trajectory by one positive integer
factor ``k`` (every segment duration ``T_i`` becomes ``k * T_i``) leaves the
path shape, the travel envelope and C0/C1 continuity untouched, while every
velocity peak scales by ``1/k`` and every acceleration peak by ``1/k**2``.

A scale ``k`` therefore qualifies iff

* ``k * T_i / cycle_duration`` is a positive integer for every segment — with
  ``T_i / cycle_duration = p_i / q_i`` in lowest terms this holds iff ``k`` is
  a multiple of ``Q = lcm(q_1, …, q_n)``; and
* for every joint, ``peak_v / k <= velocity_limit`` and
  ``peak_a / k**2 <= acceleration_limit`` (equal to the limit qualifies),
  i.e. ``k >= K_min`` with ``K_min`` computed by exact rational arithmetic.

The feasible scales are exactly the multiples of ``Q`` that are ``>= K_min``,
so the least feasible scale is ``Q * ceil(K_min / Q)`` — proving by
construction that no smaller positive integer scale qualifies. All arithmetic
is exact (``Fraction``/integer only): no floating-point tolerance and no
sampling anywhere.
"""

from __future__ import annotations

from fractions import Fraction
from math import gcd, isqrt
from typing import Dict, List, Tuple

from .audit import (
    _check_continuity,
    _check_structure,
    format_exact,
    segment_peaks,
    to_fraction,
    travel_violations,
)
from .schemas import (
    JointPeak,
    RetimeRequest,
    RetimeResponse,
    RetimeProof,
    RetimeSegmentPlan,
)

# Stable 409 reason codes (checked in this order).
REASON_TRAVEL = "travel_out_of_bounds"
REASON_ZERO_VELOCITY = "zero_velocity_limit"
REASON_ZERO_ACCELERATION = "zero_acceleration_limit"
REASON_SCALE = "scale_exceeds_max"


class RetimeConflictError(Exception):
    """A structurally valid trajectory that no allowed scale can fix.

    Carries a stable machine-readable ``reason`` plus a human message and
    structured extras so the HTTP layer can return a 409 clients can act on.
    """

    def __init__(self, reason: str, msg: str, extra: Dict | None = None):
        super().__init__(msg)
        self.reason = reason
        self.msg = msg
        self.extra = extra or {}


def ceil_sqrt_ratio(value: Fraction) -> int:
    """Smallest non-negative integer ``k`` with ``k**2 >= value`` (exact)."""
    if value <= 0:
        return 0
    numerator = value.numerator
    denominator = value.denominator
    # isqrt(floor(value)) is at most one below the true ceiling.
    k = isqrt(numerator // denominator)
    while k * k * denominator < numerator:
        k += 1
    return k


def _lcm(a: int, b: int) -> int:
    return a // gcd(a, b) * b


def _ceil_div(a: int, b: int) -> int:
    """Ceiling of a/b for positive integers."""
    return -(-a // b)


def cycle_multiple(durations: List[Fraction], cycle: Fraction) -> int:
    """Least ``Q`` such that ``k * T_i / cycle`` is integral for every segment
    whenever ``k`` is a multiple of ``Q`` (and only then)."""
    multiple = 1
    for duration in durations:
        ratio = duration / cycle  # p/q in lowest terms
        multiple = _lcm(multiple, ratio.denominator)
    return multiple


def limit_min_scale(
    peaks: List[Tuple[Fraction, Fraction]],
    velocity_limits: List[Fraction],
    acceleration_limits: List[Fraction],
) -> int:
    """Least positive integer ``k`` satisfying every velocity/acceleration
    limit after retiming by ``k`` (exact; equality with a limit qualifies)."""
    k_min = 1
    for (peak_v, peak_a), v_limit, a_limit in zip(
        peaks, velocity_limits, acceleration_limits
    ):
        if peak_v > 0:
            # k >= peak_v / v_limit, i.e. k >= ceil(peak_v / v_limit).
            required = _ceil_div(
                peak_v.numerator * v_limit.denominator,
                peak_v.denominator * v_limit.numerator,
            )
            k_min = max(k_min, required)
        if peak_a > 0:
            # k**2 >= peak_a / a_limit, i.e. k >= ceil(sqrt(peak_a / a_limit)).
            k_min = max(k_min, ceil_sqrt_ratio(peak_a / a_limit))
    return k_min


def run_retime(req: RetimeRequest) -> RetimeResponse:
    """Validate, then pick the smallest qualifying uniform integer scale."""
    _check_structure(req)

    durations = [to_fraction(seg.duration) for seg in req.segments]
    positions = [
        {
            joint.id: [to_fraction(x) for x in seg.control_positions[joint.id]]
            for joint in req.joints
        }
        for seg in req.segments
    ]
    _check_continuity(req, durations, positions)

    # Retiming cannot move positions: a travel violation can never be fixed
    # by slowing down, so it conflicts with every possible scale.
    travel: List[dict] = []
    for seg_index in range(len(req.segments)):
        for joint_index, joint in enumerate(req.joints):
            for violation in travel_violations(
                seg_index, joint_index, joint, positions[seg_index][joint.id]
            ):
                travel.append(violation.model_dump())
    if travel:
        raise RetimeConflictError(
            REASON_TRAVEL,
            "trajectory leaves the travel envelope; no uniform time scale "
            "can change the path",
            {"violations": travel},
        )

    # Exact peaks of the original (unscaled) continuous trajectory.
    peaks: List[Tuple[Fraction, Fraction]] = []
    for joint_index, joint in enumerate(req.joints):
        peak_v = Fraction(0)
        peak_a = Fraction(0)
        for seg_index in range(len(req.segments)):
            seg_peak_v, seg_peak_a = segment_peaks(
                positions[seg_index][joint.id], durations[seg_index]
            )
            peak_v = max(peak_v, seg_peak_v)
            peak_a = max(peak_a, seg_peak_a)
        peaks.append((peak_v, peak_a))

    velocity_limits = [to_fraction(j.velocity_limit) for j in req.joints]
    acceleration_limits = [to_fraction(j.acceleration_limit) for j in req.joints]

    # A non-zero peak can be shrunk by slowing down but never brought to
    # zero, so a zero limit with a non-zero peak is unsatisfiable.
    for reason, limits, peak_index, label in (
        (REASON_ZERO_VELOCITY, velocity_limits, 0, "velocity"),
        (REASON_ZERO_ACCELERATION, acceleration_limits, 1, "acceleration"),
    ):
        offenders = [
            {
                "joint_index": i,
                "joint": req.joints[i].id,
                f"peak_{label}": format_exact(peaks[i][peak_index]),
            }
            for i in range(len(req.joints))
            if limits[i] == 0 and peaks[i][peak_index] > 0
        ]
        if offenders:
            raise RetimeConflictError(
                reason,
                f"{label} limit is zero but the trajectory has a non-zero "
                f"peak {label}; no finite scale can satisfy it",
                {"joints": offenders},
            )

    cycle = to_fraction(req.cycle_duration)
    multiple = cycle_multiple(durations, cycle)
    k_min = limit_min_scale(peaks, velocity_limits, acceleration_limits)
    scale = multiple * _ceil_div(k_min, multiple)
    if scale > req.max_scale:
        raise RetimeConflictError(
            REASON_SCALE,
            f"smallest qualifying uniform scale {scale} exceeds "
            f"max_scale {req.max_scale}",
            {"required_scale": str(scale), "max_scale": req.max_scale},
        )

    scale_fraction = Fraction(scale)
    plan: List[RetimeSegmentPlan] = []
    total = Fraction(0)
    for seg_index, duration in enumerate(durations):
        new_duration = duration * scale
        total += new_duration
        cycles = new_duration / cycle
        plan.append(
            RetimeSegmentPlan(
                segment_index=seg_index,
                duration=format_exact(new_duration),
                cycles=cycles.numerator,  # integral by construction
            )
        )

    return RetimeResponse(
        scale=scale,
        cycle_duration=format_exact(cycle),
        segments=plan,
        total_duration=format_exact(total),
        total_cycles=(total / cycle).numerator,
        joints=[
            JointPeak(
                joint_index=i,
                joint=joint.id,
                peak_velocity=format_exact(peaks[i][0] / scale_fraction),
                peak_acceleration=format_exact(peaks[i][1] / scale_fraction**2),
            )
            for i, joint in enumerate(req.joints)
        ],
        proof=RetimeProof(
            cycle_multiple=multiple,
            limit_min_scale=k_min,
            minimal=True,
        ),
    )


__all__ = [
    "RetimeConflictError",
    "ceil_sqrt_ratio",
    "cycle_multiple",
    "limit_min_scale",
    "run_retime",
]
