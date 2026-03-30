// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

struct IntakeBottomRollerIOInputs {
  units::turn_t motorPosition{0_tr};
  units::turns_per_second_t motorVelocity{0_tps};

  units::volt_t appliedVolts{0_V};
  units::ampere_t statorCurrent{0_A};
  units::ampere_t supplyCurrent{0_A};

  units::second_t timestamp{0_s};
};

class IntakeBottomRollerIO {
public:
  virtual ~IntakeBottomRollerIO() = default;

  virtual void UpdateInputs(IntakeBottomRollerIOInputs &inputs) = 0;

  virtual void SetVoltage(units::volt_t voltage) = 0;
  virtual void SetCurrent(units::ampere_t current) = 0;
  virtual void SetVelocity(units::turns_per_second_t rps) = 0;
  virtual void Stop() = 0;
};
