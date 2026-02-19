// Team 5687 2026
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

struct FlywheelIOInputs {
  units::turn_t leftMotorPosition{0_tr};
  units::turn_t rightMotorPosition{0_tr};

  units::turns_per_second_t leftMotorVelocity{0_tps};
  units::turns_per_second_t rightMotorVelocity{0_tps};

  units::volt_t leftAppliedVolts{0_V};
  units::volt_t rightAppliedVolts{0_V};

  units::ampere_t leftStatorCurrent{0_A};
  units::ampere_t rightStatorCurrent{0_A};

  units::ampere_t leftSupplyCurrent{0_A};
  units::ampere_t rightSupplyCurrent{0_A};

  units::second_t timestamp{0_s};
};

class FlywheelIO {
public:
  virtual ~FlywheelIO() = default;

  virtual void UpdateInputs(FlywheelIOInputs &inputs) = 0;

  virtual void SetMotorVelocity(units::turns_per_second_t leftRPS,
                                units::turns_per_second_t rightRPS) = 0;

  virtual void SetVoltage(units::volt_t voltage) = 0;
};
