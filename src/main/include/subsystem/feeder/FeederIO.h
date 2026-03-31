// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

struct FeederIOInputs {
  // These are from the leader
  units::turn_t motorPosition{0_tr};
  units::turns_per_second_t motorVelocity{0_tps};

  units::volt_t appliedVolts{0_V};
  units::ampere_t statorCurrent{0_A};
  units::ampere_t supplyCurrent{0_A};
  units::ampere_t followerStatorCurrent{0_A};
  units::ampere_t followerSupplyCurrent{0_A};
  units::volt_t followerAppliedVolts{0_V};
  units::second_t timestamp{0_s};
  bool fuelDetected{false};
};

class FeederIO {
public:
  virtual ~FeederIO() = default;

  virtual void UpdateInputs(FeederIOInputs &inputs) = 0;

  virtual void SetVoltage(units::volt_t voltage) = 0;
  virtual void SetCurrent(units::ampere_t current) = 0;
  virtual void SetVelocity(units::turns_per_second_t rps) = 0;
  virtual void SetPosition(units::turn_t position) = 0;
  virtual void Stop() = 0;
};
