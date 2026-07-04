// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT
//
// Tests for PositionUnwrapper (continuous position from wrapped feedback).

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "robstride_driver/position_unwrapper.hpp"

namespace robstride {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kSpan = 8.0 * kPi;  // RS02: +-4 pi

TEST(PositionUnwrapper, FirstSamplePassesThrough) {
  PositionUnwrapper unwrapper(kSpan);
  EXPECT_FALSE(unwrapper.position().has_value());
  EXPECT_DOUBLE_EQ(unwrapper.Update(1.5), 1.5);
  EXPECT_DOUBLE_EQ(unwrapper.position().value(), 1.5);
}

TEST(PositionUnwrapper, SmallDeltasUnchanged) {
  PositionUnwrapper unwrapper(kSpan);
  unwrapper.Update(0.0);
  EXPECT_DOUBLE_EQ(unwrapper.Update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(unwrapper.Update(-2.0), -2.0);
}

TEST(PositionUnwrapper, PositiveWrap) {
  PositionUnwrapper unwrapper(kSpan);
  // Rotating forward: raw jumps from just below +4 pi to just above -4 pi.
  unwrapper.Update(4.0 * kPi - 0.1);
  const double continuous = unwrapper.Update(-4.0 * kPi + 0.1);
  EXPECT_NEAR(continuous, 4.0 * kPi + 0.1, 1e-9);
}

TEST(PositionUnwrapper, NegativeWrap) {
  PositionUnwrapper unwrapper(kSpan);
  // Rotating backward: raw jumps from just above -4 pi to just below +4 pi.
  unwrapper.Update(-4.0 * kPi + 0.1);
  const double continuous = unwrapper.Update(4.0 * kPi - 0.1);
  EXPECT_NEAR(continuous, -4.0 * kPi - 0.1, 1e-9);
}

TEST(PositionUnwrapper, MultipleRevolutionsAccumulate) {
  PositionUnwrapper unwrapper(kSpan);
  // Simulate constant forward rotation at 2 rad per sample for 3 full
  // wrap cycles and check the output stays continuous and monotonic.
  double raw = 0.0;
  double previous = unwrapper.Update(raw);
  double travelled = 0.0;
  const double step = 2.0;
  const int steps = static_cast<int>(3.0 * kSpan / step);
  for (int i = 0; i < steps; ++i) {
    raw += step;
    if (raw > 4.0 * kPi) {
      raw -= kSpan;
    }
    const double continuous = unwrapper.Update(raw);
    EXPECT_NEAR(continuous - previous, step, 1e-9);
    previous = continuous;
    travelled += step;
  }
  EXPECT_NEAR(previous, travelled, 1e-9);
}

TEST(PositionUnwrapper, ResetForgetsOffset) {
  PositionUnwrapper unwrapper(kSpan);
  unwrapper.Update(4.0 * kPi - 0.1);
  unwrapper.Update(-4.0 * kPi + 0.1);  // wrapped once
  unwrapper.Reset();
  EXPECT_FALSE(unwrapper.position().has_value());
  EXPECT_DOUBLE_EQ(unwrapper.Update(1.0), 1.0);
}

TEST(PositionUnwrapper, LimitsConstructorUsesSymmetricRange) {
  const ActuatorLimits& limits = GetActuatorLimits(ActuatorType::kRs02);
  PositionUnwrapper unwrapper(limits);
  unwrapper.Update(limits.position - 0.1);
  const double continuous = unwrapper.Update(-limits.position + 0.1);
  EXPECT_NEAR(continuous, limits.position + 0.1, 1e-9);
}

}  // namespace
}  // namespace robstride
