"""Exact continuous-time auditing of piecewise cubic Bézier trajectories.

All arithmetic is exact: decimal inputs are converted to ``Fraction`` so
decimal-equivalent spellings (``0.5`` / ``0.50`` / ``5E-1``) always produce
identical peaks, violation ordering and approval results.

Velocity and acceleration are adjudicated analytically over the whole
curve, never by sampling:

* position over physical time ``s`` is ``B(t)`` with ``t = s / T``;
* velocity ``dP/ds`` is a quadratic Bézier in ``t`` with coefficients
  ``v_i = 3 (c_{i+1} - c_i) / T``;
* acceleration ``d²P/ds²`` is a linear Bézier in ``t`` with coefficients
  ``a_i = 2 (v_{i+1} - v_i) / T``.

Therefore ``|acceleration|`` peaks at ``t ∈ {0, 1}`` and ``|velocity|``
peaks at ``t ∈ {0, 1}`` or at the unique interior root of the
acceleration — all computed with exact rational arithmetic.
"""

from __future__ import annotations

from decimal import Decimal
from fractions import Fraction
from typing import Dict, List, Tuple

from .schemas import AuditRequest, AuditResponse, JointPeak, Violation

# Stable constraint ordering for violations: travel, then velocity, then
# acceleration (the order the limits are stated in the API contract).
CONSTRAINT_ORDER = {"travel": 0, "velocity": 1, "acceleration": 2}
BOUND_ORDER = {"lower": 0, "upper": 1}


class AuditInputError(Exception):
    """Well-formed JSON that violates trajectory semantics.

    Carries FastAPI-style error dicts (``loc`` / ``msg`` / ``type``) so the
    HTTP layer can return a 422 whose offending fields clients can locate.
    """

    def __init__(self, errors: List[dict]):
        super().__init__("trajectory rejected")
        self.errors = errors


def to_fraction(value: Decimal) -> Fraction:
    """Exact decimal -> rational conversion."""
    return Fraction(value)


def format_exact(value: Fraction) -> str:
    """Deterministic exact string for a rational number.

    Terminating decimals render as plain decimals (``"2.5"``); anything
    else renders as a reduced fraction (``"1/3"``) so no precision is lost.
    """
    if value.denominator == 1:
        return str(value.numerator)
    denominator = value.denominator
    twos = 0
    while denominator % 2 == 0:
        denominator //= 2
        twos += 1
    fives = 0
    while denominator % 5 == 0:
        denominator //= 5
        fives += 1
    if denominator != 1:
        return f"{value.numerator}/{value.denominator}"
    places = max(twos, fives)
    scaled = value.numerator * (2 ** (places - twos)) * (5 ** (places - fives))
    sign = "-" if scaled < 0 else ""
    digits = str(abs(scaled)).rjust(places + 1, "0")
    integer_part = digits[:-places]
    fraction_part = digits[-places:].rstrip("0")
    if not fraction_part:
        return sign + integer_part
    return f"{sign}{integer_part}.{fraction_part}"


def segment_peaks(control: List[Fraction], duration: Fraction) -> Tuple[Fraction, Fraction]:
    """Exact peak |velocity| and |acceleration| of one cubic Bézier segment.

    ``control`` is ``[c0, c1, c2, c3]`` and ``duration`` the physical
    segment time ``T > 0``. Returns ``(peak_|velocity|, peak_|acceleration|)``.
    """
    c0, c1, c2, c3 = control
    v0 = 3 * (c1 - c0) / duration
    v1 = 3 * (c2 - c1) / duration
    v2 = 3 * (c3 - c2) / duration
    a0 = 2 * (v1 - v0) / duration
    a1 = 2 * (v2 - v1) / duration

    peak_velocity = max(abs(v0), abs(v2))
    if a0 * a1 < 0:
        # Acceleration is linear and crosses zero exactly once inside (0, 1);
        # that interior stationary point is the only possible interior
        # extremum of the quadratic velocity.
        t = a0 / (a0 - a1)
        u = 1 - t
        velocity_at = u * u * v0 + 2 * u * t * v1 + t * t * v2
        peak_velocity = max(peak_velocity, abs(velocity_at))
    peak_acceleration = max(abs(a0), abs(a1))
    return peak_velocity, peak_acceleration


def _check_structure(req: AuditRequest) -> None:
    """Duplicate joint ids and per-segment joint coverage."""
    errors: List[dict] = []
    seen: Dict[str, int] = {}
    for index, joint in enumerate(req.joints):
        if joint.id in seen:
            errors.append(
                {
                    "loc": ["body", "joints", index, "id"],
                    "msg": f"duplicate joint id {joint.id!r}",
                    "type": "value_error.duplicate_joint_id",
                }
            )
        else:
            seen[joint.id] = index
    joint_ids = set(seen)
    for seg_index, segment in enumerate(req.segments):
        provided = set(segment.control_positions)
        for joint in req.joints:
            if joint.id not in provided:
                errors.append(
                    {
                        "loc": ["body", "segments", seg_index, "control_positions"],
                        "msg": f"missing control positions for joint {joint.id!r}",
                        "type": "value_error.missing_joint",
                    }
                )
        for key in sorted(provided - joint_ids):
            errors.append(
                {
                    "loc": ["body", "segments", seg_index, "control_positions", key],
                    "msg": f"control positions supplied for unknown joint {key!r}",
                    "type": "value_error.unknown_joint",
                }
            )
    if errors:
        raise AuditInputError(errors)


def _check_continuity(
    req: AuditRequest,
    durations: List[Fraction],
    positions: List[Dict[str, List[Fraction]]],
) -> None:
    """Exact C0 (position) and C1 (endpoint velocity) continuity."""
    errors: List[dict] = []
    for seg_index in range(len(req.segments) - 1):
        current = positions[seg_index]
        following = positions[seg_index + 1]
        t_current = durations[seg_index]
        t_following = durations[seg_index + 1]
        for joint in req.joints:
            jid = joint.id
            cur = current[jid]
            nxt = following[jid]
            if cur[3] != nxt[0]:
                errors.append(
                    {
                        "loc": ["body", "segments", seg_index + 1, "control_positions", jid, 0],
                        "msg": (
                            f"position discontinuity between segments {seg_index} and "
                            f"{seg_index + 1} for joint {jid!r}: segment {seg_index} ends "
                            f"at {format_exact(cur[3])} but segment {seg_index + 1} starts "
                            f"at {format_exact(nxt[0])}"
                        ),
                        "type": "continuity.position",
                    }
                )
            end_velocity = 3 * (cur[3] - cur[2]) / t_current
            start_velocity = 3 * (nxt[1] - nxt[0]) / t_following
            if end_velocity != start_velocity:
                errors.append(
                    {
                        "loc": ["body", "segments", seg_index + 1, "control_positions", jid, 1],
                        "msg": (
                            f"velocity discontinuity between segments {seg_index} and "
                            f"{seg_index + 1} for joint {jid!r}: segment {seg_index} ends "
                            f"with velocity {format_exact(end_velocity)} but segment "
                            f"{seg_index + 1} starts with velocity "
                            f"{format_exact(start_velocity)}"
                        ),
                        "type": "continuity.velocity",
                    }
                )
    if errors:
        raise AuditInputError(errors)


def run_audit(req: AuditRequest) -> AuditResponse:
    """Validate continuity, then adjudicate the whole continuous trajectory."""
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

    violations: List[Violation] = []
    peak_velocity = [Fraction(0)] * len(req.joints)
    peak_acceleration = [Fraction(0)] * len(req.joints)

    for seg_index in range(len(req.segments)):
        duration = durations[seg_index]
        for joint_index, joint in enumerate(req.joints):
            jid = joint.id
            cps = positions[seg_index][jid]
            low = to_fraction(joint.travel.min)
            high = to_fraction(joint.travel.max)

            # Travel is a closed interval: equal to a bound is acceptable.
            below = [i for i, c in enumerate(cps) if c < low]
            if below:
                worst = min(below, key=lambda i: (cps[i], i))
                violations.append(
                    Violation(
                        segment_index=seg_index,
                        joint_index=joint_index,
                        joint=jid,
                        constraint="travel",
                        bound="lower",
                        control_point_index=worst,
                        limit=format_exact(low),
                        value=format_exact(cps[worst]),
                    )
                )
            above = [i for i, c in enumerate(cps) if c > high]
            if above:
                worst = max(above, key=lambda i: (cps[i], -i))
                violations.append(
                    Violation(
                        segment_index=seg_index,
                        joint_index=joint_index,
                        joint=jid,
                        constraint="travel",
                        bound="upper",
                        control_point_index=worst,
                        limit=format_exact(high),
                        value=format_exact(cps[worst]),
                    )
                )

            seg_peak_v, seg_peak_a = segment_peaks(cps, duration)
            if seg_peak_v > peak_velocity[joint_index]:
                peak_velocity[joint_index] = seg_peak_v
            if seg_peak_a > peak_acceleration[joint_index]:
                peak_acceleration[joint_index] = seg_peak_a

            # Strictly greater than the limit violates; equal passes.
            v_limit = to_fraction(joint.velocity_limit)
            a_limit = to_fraction(joint.acceleration_limit)
            if seg_peak_v > v_limit:
                violations.append(
                    Violation(
                        segment_index=seg_index,
                        joint_index=joint_index,
                        joint=jid,
                        constraint="velocity",
                        bound=None,
                        control_point_index=None,
                        limit=format_exact(v_limit),
                        value=format_exact(seg_peak_v),
                    )
                )
            if seg_peak_a > a_limit:
                violations.append(
                    Violation(
                        segment_index=seg_index,
                        joint_index=joint_index,
                        joint=jid,
                        constraint="acceleration",
                        bound=None,
                        control_point_index=None,
                        limit=format_exact(a_limit),
                        value=format_exact(seg_peak_a),
                    )
                )

    violations.sort(
        key=lambda v: (
            v.segment_index,
            v.joint_index,
            CONSTRAINT_ORDER[v.constraint],
            BOUND_ORDER.get(v.bound or "", 0),
        )
    )

    return AuditResponse(
        approved=not violations,
        joints=[
            JointPeak(
                joint_index=i,
                joint=joint.id,
                peak_velocity=format_exact(peak_velocity[i]),
                peak_acceleration=format_exact(peak_acceleration[i]),
            )
            for i, joint in enumerate(req.joints)
        ],
        violations=violations,
    )
