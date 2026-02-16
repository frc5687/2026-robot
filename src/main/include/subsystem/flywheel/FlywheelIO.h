// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>

struct FlywheelIOInputs {
  units::turns_per_second_t motorVelocity{0_tps};
  units::radians_per_second_squared_t motorAcceleration{0_rad_per_s_sq};

  units::second_t timestamp{0_s};
};

class FlywheelIO {
 public:
  virtual ~FlywheelIO() = default;
  virtual void UpdateInputs(FlywheelIOInputs& inputs) = 0;
};
