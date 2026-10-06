"""Unit tests for the exact audit mathematics."""

from decimal import Decimal
from fractions import Fraction

from app.audit import format_exact, segment_peaks, to_fraction

F = Fraction


class TestSegmentPeaks:
    def test_interior_velocity_extremum(self):
        # V(t) = 6t(1-t): peak 3/2 at t=1/2, acceleration ±6.
        pv, pa = segment_peaks([F(0), F(0), F(1), F(1)], F(1))
        assert pv == F(3, 2)
        assert pa == F(6)

    def test_constant_velocity(self):
        pv, pa = segment_peaks([F(0), F(1), F(2), F(3)], F(1))
        assert pv == F(3)
        assert pa == F(0)

    def test_peak_at_segment_endpoints(self):
        # Monotone velocity: peak at t=1, acceleration constant.
        pv, pa = segment_peaks([F(0), F(0), F(0), F(1)], F(1))
        assert pv == F(3)
        assert pa == F(6)

    def test_duration_scales_derivatives(self):
        pv, pa = segment_peaks([F(0), F(0), F(1), F(1)], F(2))
        assert pv == F(3, 4)
        assert pa == F(3, 2)

    def test_nonterminating_exact_peak(self):
        # v coefficients 0, 1/3, 0 -> peak velocity exactly 1/6 at t=1/2,
        # acceleration ±20/27. Neither is a terminating decimal.
        pv, pa = segment_peaks([F(0), F(0), F(1, 10), F(1, 10)], F(9, 10))
        assert pv == F(1, 6)
        assert pa == F(20, 27)

    def test_negative_direction_uses_absolute_value(self):
        pv, pa = segment_peaks([F(0), F(0), F(-1), F(-1)], F(1))
        assert pv == F(3, 2)
        assert pa == F(6)


class TestFormatExact:
    def test_integers(self):
        assert format_exact(F(0)) == "0"
        assert format_exact(F(4)) == "4"
        assert format_exact(F(-7)) == "-7"

    def test_terminating_decimals(self):
        assert format_exact(F(1, 2)) == "0.5"
        assert format_exact(F(5, 2)) == "2.5"
        assert format_exact(F(-1, 8)) == "-0.125"
        assert format_exact(F(1, 20)) == "0.05"

    def test_nonterminating_renders_as_fraction(self):
        assert format_exact(F(1, 3)) == "1/3"
        assert format_exact(F(-20, 27)) == "-20/27"
        assert format_exact(F(1, 6)) == "1/6"


class TestDecimalEquivalence:
    def test_equivalent_spellings_are_the_same_rational(self):
        spellings = ["0.5", "0.50", "0.500", "5E-1", "50E-2", "+0.5", "0.5e0"]
        fractions = {to_fraction(Decimal(s)) for s in spellings}
        assert fractions == {F(1, 2)}

    def test_negative_zero_is_zero(self):
        assert to_fraction(Decimal("-0")) == F(0)
        assert to_fraction(Decimal("-0.00")) == to_fraction(Decimal("0.0"))
