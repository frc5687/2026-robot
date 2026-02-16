#pragma once

#include <units/time.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include "utils/Utils.h"

struct HoodState {
    units::second_t timestamp{0_s};
    units::radian_t angle{0_rad};
    units::radians_per_second_t velocity{0_rad_per_s};

    // This moves so slow that we probably dont need accel
    HoodState Extrapolate(units::second_t dt) const {
        HoodState future = *this;
        future.timestamp = timestamp + dt;
        future.angle = angle + (velocity * dt);
        future.velocity = velocity;
        return future;
    }

    static HoodState Interpolate(const HoodState& start, const HoodState& end, double t) {
        HoodState interpolated;
        interpolated.timestamp = Lerp(start.timestamp, end.timestamp, t);
        interpolated.angle = Lerp(start.angle, end.angle, t);
        interpolated.velocity = Lerp(start.velocity, end.velocity, t);
        return interpolated;
    }
};
