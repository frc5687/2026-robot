// Team 5687 2026

#pragma once

#include <frc/filter/LinearFilter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <memory>

#include "FlywheelIO.h"
#include "FlywheelState.h"
#include "subsystem/LoggedSubsystem.h"

class FlywheelSubsystem : public LoggedSubsystem {
public:
  explicit FlywheelSubsystem(std::unique_ptr<FlywheelIO> io);

  void SetRPM(units::revolutions_per_minute_t desiredRPM);
  bool AtSetpoint() const;

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  void UpdateInputs();
  void LogTelemetry();

private:
  std::unique_ptr<FlywheelIO> m_io;
  FlywheelIOInputs m_inputs{};

  units::revolutions_per_minute_t m_desiredRPM{0_rpm};
  frc::LinearFilter<units::revolutions_per_minute_t> m_filter =
      frc::LinearFilter<units::revolutions_per_minute_t>::MovingAverage(6);
  units::revolutions_per_minute_t m_filteredRPM{0_rpm};

  FlywheelState m_state{};

  frc2::sysid::SysIdRoutine m_sysIdRoutine;
  void SysIdDrive(units::volt_t voltage);
  void SysIdLog(frc::sysid::SysIdRoutineLog *log);

  static units::turns_per_second_t
  MechanismRPMToMotorRPS(units::revolutions_per_minute_t rpm);
  static units::revolutions_per_minute_t
  MotorRPSToMechanismRPM(units::turns_per_second_t rps);
  static units::radians_per_second_t
  MotorRPSToMechanismRadPerSec(units::turns_per_second_t rps);

  void UpdateFlywheelState(FlywheelState &state,
                           units::turns_per_second_t motorVelocity,
                           units::second_t timestamp);
};
