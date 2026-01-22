#pragma once

#include <units/angular_velocity.h>
#include <units/time.h>

struct FlywheelIOInputs {
  units::revolutions_per_minute_t flywheelVelocity{0_rpm};
  units::turns_per_second_t motorVelocity{0_tps};

  units::second_t timestamp{0_s};
};

class FlywheelIO {
public:
  virtual ~FlywheelIO() = default;

  virtual void UpdateInputs(FlywheelIOInputs& inputs) = 0;
  virtual void SetFlywheelRPM(units::revolutions_per_minute_t desiredRpm) = 0;
};
