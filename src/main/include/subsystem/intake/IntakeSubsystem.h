// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/IntakeIO.h"

class IntakeSubsystem : public LoggedSubsystem {
public:
  explicit IntakeSubsystem(std::unique_ptr<IntakeIO> io);

  void SetVoltage(units::volt_t voltage);
  void Stop();

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<IntakeIO> m_io;
  IntakeIOInputs m_inputs{};
};
