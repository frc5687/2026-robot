// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/torque.h>

#include "utils/Utils.h"

struct TurretState {
  units::second_t timestamp{0.0};
  units::radian_t angle{0.0_rad};
  units::radians_per_second_t velocity{0.0_rad_per_s};
  units::radians_per_second_squared_t acceleration{0.0_rad_per_s_sq};

  units::newton_meter_t torque{0_Nm};

  TurretState Extrapolate(units::second_t dt) const {
    TurretState future = *this;
    future.timestamp = timestamp + dt;
    future.angle = angle + (velocity * dt) + (0.5 * acceleration * dt * dt);
    future.velocity = velocity + (acceleration * dt);
    return future;
  }

  static TurretState Interpolate(const TurretState& start,
                                 const TurretState& end, double t) {
    TurretState interpolated;
    interpolated.timestamp = Lerp(start.timestamp, end.timestamp, t);
    interpolated.angle = Lerp(start.angle, end.angle, t);
    interpolated.velocity = Lerp(start.velocity, end.velocity, t);
    // interpolated.timestamp =
    //     start.timestamp + (end.timestamp - start.timestamp) * t;
    // interpolated.angle = start.angle + (end.angle - start.angle) * t;
    // interpolated.velocity =
    //     start.velocity + (end.velocity - start.velocity) * t;
    interpolated.acceleration = end.acceleration;
    interpolated.torque = end.torque;
    return interpolated;
  }
};
