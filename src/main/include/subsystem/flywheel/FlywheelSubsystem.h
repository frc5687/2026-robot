// Team 5687 2026

#pragma once

#include <frc/filter/LinearFilter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <memory>

#include "Constants.h"
#include "subsystem/LoggedSubsystem.h"
#include "subsystem/flywheel/FlywheelIO.h"
#include "subsystem/flywheel/FlywheelState.h"

class FlywheelSubsystem : public LoggedSubsystem {
public:
  explicit FlywheelSubsystem(std::unique_ptr<FlywheelIO> io);

  void SetRPM(units::revolutions_per_minute_t desiredRPMLeft,
              units::revolutions_per_minute_t desiredRPMRight);

  bool AtSetpoint() const;

  const FlywheelState &GetLeftState() const { return m_leftState; }
  const FlywheelState &GetRightState() const { return m_rightState; }

  units::revolutions_per_minute_t GetFilteredLeftRPM() const {
    return m_filteredLeft;
  }
  units::revolutions_per_minute_t GetFilteredRightRPM() const {
    return m_filteredRight;
  }

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  static units::turns_per_second_t
  MechanismRPMToMotorRPS(units::revolutions_per_minute_t rpm);
  static units::revolutions_per_minute_t
  MotorRPSToMechanismRPM(units::turns_per_second_t rps);
  static units::radians_per_second_t
  MotorRPSToMechanismRadPerSec(units::turns_per_second_t rps);

  void UpdateFlywheelState(FlywheelState &state,
                           units::turns_per_second_t motorVelocity,
                           units::second_t timestamp);

  void SysIdDrive(units::volt_t voltage);
  void SysIdLog(frc::sysid::SysIdRoutineLog *log);

  std::unique_ptr<FlywheelIO> m_io;
  FlywheelIOInputs m_inputs{};

  units::revolutions_per_minute_t m_desiredRPMLeft{0_rpm};
  units::revolutions_per_minute_t m_desiredRPMRight{0_rpm};

  FlywheelState m_leftState{};
  FlywheelState m_rightState{};

  frc::LinearFilter<units::revolutions_per_minute_t> m_filterLeft =
      frc::LinearFilter<units::revolutions_per_minute_t>::SinglePoleIIR(
          Constants::Flywheel::kFilterTime.value(),
          Constants::Flywheel::kFilterPeriod);
  frc::LinearFilter<units::revolutions_per_minute_t> m_filterRight =
      frc::LinearFilter<units::revolutions_per_minute_t>::SinglePoleIIR(
          Constants::Flywheel::kFilterTime.value(),
          Constants::Flywheel::kFilterPeriod);

  units::revolutions_per_minute_t m_filteredLeft{0_rpm};
  units::revolutions_per_minute_t m_filteredRight{0_rpm};

  frc2::sysid::SysIdRoutine m_sysIdRoutine;
};
