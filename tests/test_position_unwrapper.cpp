// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Tests for PositionUnwrapper (continuous position from wrapped feedback).

#include <gtest/gtest.h>

#include "robstride_driver/position_unwrapper.hpp"

namespace robstride {
namespace {

const ActuatorLimits& Rs02() { return get_actuator_limits(ActuatorType::Rs02); }

/// Total width of the wrapped encoding range (RS02: 8 pi).
double WrapSpan() { return 2.0 * Rs02().position; }

TEST(PositionUnwrapper, FirstSamplePassesThrough) {
  PositionUnwrapper unwrapper(WrapSpan());
  EXPECT_FALSE(unwrapper.position().has_value());
  EXPECT_DOUBLE_EQ(unwrapper.update(1.5), 1.5);
  EXPECT_DOUBLE_EQ(unwrapper.position().value(), 1.5);
}

TEST(PositionUnwrapper, SmallDeltasUnchanged) {
  PositionUnwrapper unwrapper(WrapSpan());
  unwrapper.update(0.0);
  EXPECT_DOUBLE_EQ(unwrapper.update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(unwrapper.update(-2.0), -2.0);
}

TEST(PositionUnwrapper, PositiveWrap) {
  PositionUnwrapper unwrapper(WrapSpan());
  // Rotating forward: raw jumps from just below +4 pi to just above -4 pi.
  unwrapper.update(Rs02().position - 0.1);
  const double continuous = unwrapper.update(-Rs02().position + 0.1);
  EXPECT_NEAR(continuous, Rs02().position + 0.1, 1e-9);
}

TEST(PositionUnwrapper, NegativeWrap) {
  PositionUnwrapper unwrapper(WrapSpan());
  // Rotating backward: raw jumps from just above -4 pi to just below +4 pi.
  unwrapper.update(-Rs02().position + 0.1);
  const double continuous = unwrapper.update(Rs02().position - 0.1);
  EXPECT_NEAR(continuous, -Rs02().position - 0.1, 1e-9);
}

TEST(PositionUnwrapper, MultipleRevolutionsAccumulate) {
  PositionUnwrapper unwrapper(WrapSpan());
  // Simulate constant forward rotation at 2 rad per sample for 3 full
  // wrap cycles and check the output stays continuous and monotonic.
  double raw = 0.0;
  double previous = unwrapper.update(raw);
  double travelled = 0.0;
  const double step = 2.0;
  const int steps = static_cast<int>(3.0 * WrapSpan() / step);
  for (int i = 0; i < steps; ++i) {
    raw += step;
    if (raw > Rs02().position) {
      raw -= WrapSpan();
    }
    const double continuous = unwrapper.update(raw);
    EXPECT_NEAR(continuous - previous, step, 1e-9);
    previous = continuous;
    travelled += step;
  }
  EXPECT_NEAR(previous, travelled, 1e-9);
}

TEST(PositionUnwrapper, ResetForgetsOffset) {
  PositionUnwrapper unwrapper(WrapSpan());
  unwrapper.update(Rs02().position - 0.1);
  unwrapper.update(-Rs02().position + 0.1);  // wrapped once
  unwrapper.reset();
  EXPECT_FALSE(unwrapper.position().has_value());
  EXPECT_DOUBLE_EQ(unwrapper.update(1.0), 1.0);
}

TEST(PositionUnwrapper, LimitsConstructorUsesSymmetricRange) {
  PositionUnwrapper unwrapper(Rs02());
  unwrapper.update(Rs02().position - 0.1);
  const double continuous = unwrapper.update(-Rs02().position + 0.1);
  EXPECT_NEAR(continuous, Rs02().position + 0.1, 1e-9);
}

}  // namespace
}  // namespace robstride
