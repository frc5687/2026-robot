// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

struct IntakeDeployerIOInputs {
  units::turn_t motorPosition{0_tr};
  units::turns_per_second_t motorVelocity{0_tps};

  units::volt_t appliedVolts{0_V};
  units::ampere_t statorCurrent{0_A};
  units::ampere_t supplyCurrent{0_A};

  bool forwardLimitHit{false};
  bool reverseLimitHit{false};

  units::second_t timestamp{0_s};
};

class IntakeDeployerIO {
public:
  virtual ~IntakeDeployerIO() = default;

  virtual void UpdateInputs(IntakeDeployerIOInputs &inputs) = 0;
  virtual void SetPosition(units::turn_t position) = 0;
  virtual void SetVoltage(units::volt_t voltage) = 0;
  virtual void ZeroPosition() = 0;
  virtual void Stop() = 0;
  virtual void SetCurrentLimits(units::ampere_t currentlimits) = 0;
};
