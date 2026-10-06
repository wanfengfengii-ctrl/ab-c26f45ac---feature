"""Request/response schemas for the trajectory audit API.

All numeric fields are parsed into exact ``Decimal`` values. Strings must
use plain decimal notation (optional sign, digits, decimal point, optional
exponent); JSON numbers are converted through their shortest decimal
representation. ``NaN``, ``Infinity``, fractions, hex literals and other
non-decimal notations are rejected with a 422.
"""

from __future__ import annotations

import re
from decimal import Decimal, InvalidOperation
from typing import Annotated, Dict, List, Optional

from pydantic import BaseModel, BeforeValidator, ConfigDict, Field, model_validator

_DECIMAL_PATTERN = re.compile(r"[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?")


def parse_decimal(value: object) -> Decimal:
    """Parse a JSON value into an exact, finite ``Decimal``.

    Accepts decimal strings (e.g. ``"0.5"``, ``"0.50"``, ``"5E-1"`` — all
    numerically equal and treated identically downstream) and JSON numbers.
    Rejects booleans, null, non-finite values and non-decimal notations.
    """
    if isinstance(value, bool):
        raise ValueError("expected a decimal number, not a boolean")
    if isinstance(value, Decimal):
        parsed = value
    elif isinstance(value, int):
        parsed = Decimal(value)
    elif isinstance(value, float):
        parsed = Decimal(str(value))
    elif isinstance(value, str):
        if not _DECIMAL_PATTERN.fullmatch(value):
            raise ValueError(f"invalid decimal notation: {value!r}")
        try:
            parsed = Decimal(value)
        except InvalidOperation:
            raise ValueError(f"invalid decimal notation: {value!r}") from None
    else:
        raise ValueError(f"expected a decimal number, got {type(value).__name__}")
    if not parsed.is_finite():
        raise ValueError("decimal value must be finite")
    return parsed


ExactDecimal = Annotated[Decimal, BeforeValidator(parse_decimal)]
PositiveDecimal = Annotated[Decimal, BeforeValidator(parse_decimal), Field(gt=0)]
NonNegativeDecimal = Annotated[Decimal, BeforeValidator(parse_decimal), Field(ge=0)]
# Exactly four control positions per joint per segment (cubic Bézier).
ControlPositions = Annotated[List[ExactDecimal], Field(min_length=4, max_length=4)]


class Travel(BaseModel):
    """Closed stroke interval [min, max] for one joint."""

    model_config = ConfigDict(extra="forbid")

    min: ExactDecimal
    max: ExactDecimal

    @model_validator(mode="after")
    def _check_order(self) -> "Travel":
        if self.min > self.max:
            raise ValueError("travel.min must be less than or equal to travel.max")
        return self


class JointSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")

    id: str = Field(min_length=1, max_length=64)
    travel: Travel
    velocity_limit: NonNegativeDecimal
    acceleration_limit: NonNegativeDecimal


class Segment(BaseModel):
    model_config = ConfigDict(extra="forbid")

    duration: PositiveDecimal
    control_positions: Dict[str, ControlPositions] = Field(min_length=1)


class AuditRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    joints: List[JointSpec] = Field(min_length=1, max_length=8)
    segments: List[Segment] = Field(min_length=1, max_length=200)


class JointPeak(BaseModel):
    joint_index: int
    joint: str
    peak_velocity: str
    peak_acceleration: str


class Violation(BaseModel):
    segment_index: int
    joint_index: int
    joint: str
    constraint: str  # "travel" | "velocity" | "acceleration"
    bound: Optional[str] = None  # for travel: "lower" | "upper"
    control_point_index: Optional[int] = None  # for travel: offending control point
    limit: str
    value: str


class AuditResponse(BaseModel):
    approved: bool
    joints: List[JointPeak]
    violations: List[Violation]
