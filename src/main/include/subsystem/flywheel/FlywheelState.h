// Team 5687 2026

#pragma once

#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>

struct FlywheelState {
  units::second_t timestamp{0.0_s};
  units::radians_per_second_t velocity{0_rad_per_s};
  units::radians_per_second_squared_t acceleration{0_rad_per_s_sq};

  FlywheelState Extrapolate(units::second_t dt) const {
    FlywheelState future = *this;
    future.timestamp = timestamp + dt;
    future.velocity =
        velocity + (acceleration * dt);  // make this torque based?
    return future;
  }

  static FlywheelState Interpolate(const FlywheelState& start,
                                   const FlywheelState& end, double t) {
    FlywheelState interpolated;
    interpolated.timestamp =
        start.timestamp + (end.timestamp - start.timestamp) * t;
    interpolated.velocity =
        start.velocity + (end.velocity - start.velocity) * t;
    interpolated.acceleration = end.acceleration;
    return interpolated;
  }
};
