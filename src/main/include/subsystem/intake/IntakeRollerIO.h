#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

struct IntakeRollerIOInputs {
  units::turn_t IntakeRollerPosition{0_tr};
  units::radians_per_second_t IntakeRollerVelocity{0_rad_per_s};
  units::meters_per_second_t intakeSpeed{0_mps};
  units::volt_t voltage{0_V};
  units::radians_per_second_t DesiredIntakeRollerVelocity{0_rad_per_s};
  units::second_t timestamp{0_s};
};

class IntakeRollerIO {
 public:
  virtual ~IntakeRollerIO() = default;
  virtual void UpdateInputs(IntakeRollerIOInputs& inputs) = 0;
  virtual void SetIntakeRPM(units::radians_per_second_t desiredVelocity) = 0;
  virtual void SetVoltage(units::volt_t voltage){}; 
};