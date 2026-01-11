#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

struct IntakeRollerIOInputs {
  units::turn_t leftMotorPosition{0_tr};
  units::turn_t rightMotorPosition{0_tr};
  units::turns_per_second_t leftMotorVelocity{0_tps};
  units::turns_per_second_t rightMotorVelocity{0_tps};

  units::meters_per_second_t intakeSpeed{0_mps};

  units::second_t timestamp{0_s};
};

class IntakeRollerIO {
 public:
  virtual ~IntakeRollerIO() = default;
  virtual void UpdateInputs(IntakeRollerIOInputs& inputs) = 0;
  virtual void SetIntakeRPM(units::revolutions_per_minute_t desiredRPM) = 0;
  virtual void SetIntakeVoltage(units::volt_t voltage) {}
};