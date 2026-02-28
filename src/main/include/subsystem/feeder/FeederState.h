// Team 5687 2026

#pragma once

#include <units/angular_velocity.h>
#include <units/time.h>

#include "utils/Utils.h"

struct FeederState {
  units::second_t timestamp{0_s};
  units::turns_per_second_t velocity{0_tps};

  FeederState Extrapolate(units::second_t dt) const {
    FeederState future = *this;
    future.timestamp = timestamp + dt;
    future.velocity = velocity;
    return future;
  }

  static FeederState Interpolate(const FeederState &start,
                                 const FeederState &end, double t) {
    FeederState interpolated;
    interpolated.timestamp = Lerp(start.timestamp, end.timestamp, t);
    interpolated.velocity = Lerp(start.velocity, end.velocity, t);
    return interpolated;
  }
};
