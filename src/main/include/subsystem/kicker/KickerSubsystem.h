// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/kicker/KickerIO.h"

class KickerSubsystem : public LoggedSubsystem {
public:
  explicit KickerSubsystem(std::unique_ptr<KickerIO> io);

  void SetVoltage(units::volt_t voltage);
  void SetVelocity(units::turns_per_second_t rps);
  void Stop();

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<KickerIO> m_io;
  KickerIOInputs m_inputs{};
};
