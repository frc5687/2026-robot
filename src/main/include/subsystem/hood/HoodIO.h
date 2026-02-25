// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

struct HoodIOInputs {
  units::turn_t motorPosition{0_tr};
  units::turns_per_second_t motorVelocity{0_tps};

  units::volt_t appliedVolts{0_V};
  units::ampere_t statorCurrent{0_A};
  units::ampere_t supplyCurrent{0_A};

  units::turn_t encoderAbsolutePosition{0_tr};

  units::second_t timestamp{0_s};
};

class HoodIO {
public:
  virtual ~HoodIO() = default;

  virtual void UpdateInputs(HoodIOInputs &inputs) = 0;

  virtual void SetPosition(units::turn_t position) = 0;
  virtual void SetVoltage(units::volt_t voltage) = 0;
  virtual void ZeroPosition() = 0;
  virtual void Stop() = 0;
};
