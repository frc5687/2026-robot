// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

struct FlywheelIOInputs {
  units::turn_t leaderMotorPosition{0_tr};
  units::turns_per_second_t leaderMotorVelocity{0_tps};
  units::volt_t leaderAppliedVolts{0_V};
  units::ampere_t leaderStatorCurrent{0_A};
  units::ampere_t leaderSupplyCurrent{0_A};

  units::ampere_t follower1StatorCurrent{0_A};
  units::ampere_t follower2StatorCurrent{0_A};
  units::ampere_t follower3StatorCurrent{0_A};

  units::second_t timestamp{0_s};
};

class FlywheelIO {
public:
  virtual ~FlywheelIO() = default;

  virtual void UpdateInputs(FlywheelIOInputs &inputs) = 0;
  virtual void SetMotorVelocity(units::turns_per_second_t rps) = 0;
  virtual void SetVoltage(units::volt_t voltage) = 0;
};
