// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/floorroller/FloorRollerIO.h"
#include "subsystem/floorroller/FloorRollerState.h"

class FloorRollerSubsystem : public LoggedSubsystem {
public:
  explicit FloorRollerSubsystem(std::unique_ptr<FloorRollerIO> io);

  void SetVoltage(units::volt_t voltage);
  void SetVelocity(units::turns_per_second_t rps);
  void Stop();

  const FloorRollerState &GetState() const { return m_state; }

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<FloorRollerIO> m_io;
  FloorRollerIOInputs m_inputs{};
  FloorRollerState m_state{};
};
