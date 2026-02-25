// Team 5687 2026

#pragma once

#include <units/angular_velocity.h>
#include <units/time.h>

#include "utils/Utils.h"

struct FloorRollerState {
  units::second_t timestamp{0_s};
  units::turns_per_second_t velocity{0_tps};

  FloorRollerState Extrapolate(units::second_t dt) const {
    FloorRollerState future = *this;
    future.timestamp = timestamp + dt;
    future.velocity = velocity;
    return future;
  }

  static FloorRollerState Interpolate(const FloorRollerState &start,
                                      const FloorRollerState &end, double t) {
    FloorRollerState interpolated;
    interpolated.timestamp = Lerp(start.timestamp, end.timestamp, t);
    interpolated.velocity = Lerp(start.velocity, end.velocity, t);
    return interpolated;
  }
};
