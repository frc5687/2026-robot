// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/feeder/FeederIO.h"
#include "subsystem/feeder/FeederState.h"

class FeederSubsystem : public LoggedSubsystem {
public:
  explicit FeederSubsystem(std::unique_ptr<FeederIO> io);

  void SetVoltage(units::volt_t voltage);
  void SetVelocity(units::turns_per_second_t rps);
  void Stop();

  const FeederState &GetState() const { return m_state; }

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<FeederIO> m_io;
  FeederIOInputs m_inputs{};
  FeederState m_state{};
};
