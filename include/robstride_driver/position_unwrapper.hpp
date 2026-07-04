// Copyright 2026 masayuki-kono
// SPDX-License-Identifier: MIT

#pragma once

#include <optional>

#include "robstride_driver/actuator_types.hpp"

namespace robstride {

/// Converts the wrapped position reported in feedback frames into a
/// continuous position.
///
/// The position field of a feedback frame (communication type 2) is encoded
/// in a fixed range (e.g. +/-4 pi for the RS02), so a continuously rotating
/// motor wraps from one end of the range to the other. This class detects
/// such jumps between consecutive samples and accumulates the corresponding
/// offset.
///
/// Wrap detection assumes the true motion between two samples is smaller
/// than half the encoding range. With the RS02 (range 8 pi, max speed
/// 44 rad/s) this holds for sampling periods up to ~280 ms.
class PositionUnwrapper {
 public:
  /// `span` is the total width of the wrapped encoding range
  /// (max - min, e.g. 8 pi for the RS02).
  explicit PositionUnwrapper(double span) : span_(span) {}

  /// Convenience constructor taking the per-model limits table
  /// (the encoding range is symmetric: [-position, +position]).
  explicit PositionUnwrapper(const ActuatorLimits& limits)
      : PositionUnwrapper(2.0 * limits.position) {}

  /// Feeds one wrapped sample and returns the continuous position.
  double Update(double wrapped) {
    if (last_wrapped_) {
      const double delta = wrapped - *last_wrapped_;
      if (delta > span_ / 2.0) {
        offset_ -= span_;
      } else if (delta < -span_ / 2.0) {
        offset_ += span_;
      }
    }
    last_wrapped_ = wrapped;
    return wrapped + offset_;
  }

  /// Latest continuous position (nullopt before the first Update call).
  std::optional<double> position() const {
    if (!last_wrapped_) {
      return std::nullopt;
    }
    return *last_wrapped_ + offset_;
  }

  /// Forgets all history; the next Update starts a new continuous track
  /// from the raw value. Call this when the motor may have been power
  /// cycled or re-zeroed.
  void Reset() {
    last_wrapped_.reset();
    offset_ = 0.0;
  }

 private:
  /// Total width of the wrapped encoding range (max - min) [rad].
  double span_;
  /// Accumulated correction added to raw samples; grows/shrinks by
  /// `span_` each time the raw position wraps around [rad].
  double offset_ = 0.0;
  /// Raw (wrapped) position of the previous Update call; nullopt until
  /// the first sample arrives [rad].
  std::optional<double> last_wrapped_;
};

}  // namespace robstride
