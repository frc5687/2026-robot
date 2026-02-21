// Team 5687 2026

#pragma once

#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerIO.h"

class IntakeTopRollerSubsystem : public LoggedSubsystem {
public:
  explicit IntakeTopRollerSubsystem(std::unique_ptr<IntakeTopRollerIO> io);

  void SetVoltage(units::volt_t voltage);
  void SetVelocity(units::turns_per_second_t rps);
  void Stop();

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<IntakeTopRollerIO> m_io;
  IntakeTopRollerIOInputs m_inputs{};
};
