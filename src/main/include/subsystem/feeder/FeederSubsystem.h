// Team 5687 2026

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/sysid/SysIdRoutine.h>
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
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  const FeederState &GetState() const { return m_state; }

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<FeederIO> m_io;
  FeederIOInputs m_inputs{};
  FeederState m_state{};

  frc2::sysid::SysIdRoutine m_sysIdRoutine;
  void SysIdDrive(units::volt_t voltage);
  void SysIdLog(frc::sysid::SysIdRoutineLog *log);
};
