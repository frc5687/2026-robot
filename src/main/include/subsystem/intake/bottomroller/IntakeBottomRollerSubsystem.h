// Team 5687 2026

#pragma once

#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/power.h>
#include <units/voltage.h>

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerIO.h"

class IntakeBottomRollerSubsystem : public LoggedSubsystem {
public:
  explicit IntakeBottomRollerSubsystem(
      std::unique_ptr<IntakeBottomRollerIO> io);

  void SetVoltage(units::volt_t voltage);
  void SetCurrent(units::ampere_t current);
  void SetVelocity(units::turns_per_second_t rps);
  void Stop();
  units::ampere_t GetElectricalCurrentDraw() const;
  units::watt_t GetElectricalPowerDraw() const;

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<IntakeBottomRollerIO> m_io;
  IntakeBottomRollerIOInputs m_inputs{};
};
