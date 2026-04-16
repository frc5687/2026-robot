// Team 5687 2026

#include "commands/shooter/ShotCalculatedSpinUpCommand.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>

#include <algorithm>

#include "subsystem/feeder/FeederConstants.h"

ShotCalculatedSpinUpCommand::ShotCalculatedSpinUpCommand(
    FlywheelSubsystem *flywheel, FeederSubsystem *feeder, FloorSubsystem *floor)
    : m_flywheel(flywheel), m_feeder(feeder), m_floor(floor) {
  AddRequirements({flywheel, feeder, floor});
  SetName("ShotCalculatedSpinUpCommand");
}

void ShotCalculatedSpinUpCommand::Initialize() {
  m_startTime = frc::Timer::GetFPGATimestamp();
  m_feeder->SetIndexingActive(false);
  m_clearanceComplete = !m_feeder->NeedsIndexing();
  if (!m_clearanceComplete) {
    m_feeder->BeginClearance();
  }
}

void ShotCalculatedSpinUpCommand::Execute() {
  auto now = frc::Timer::GetFPGATimestamp();

  if (!m_clearanceComplete) {
    m_flywheel->SetVoltage(kPreclearFlywheelReverseVoltage);
    if (m_feeder->IsCleared() ||
        now - m_startTime >= Constants::Feeder::kClearanceTimeout) {
      m_clearanceComplete = true;
      m_feeder->SetIndexed();
      m_feeder->Stop();
      m_floor->Stop();
    } else {
      m_floor->SetVoltage(kBackoffFloorVoltage);
      return;
    }
  }

  auto alliance = frc::DriverStation::GetAlliance();
  if (!alliance.has_value()) {
    m_flywheel->SetRPM(0_rpm);
    return;
  }

  auto solution = m_shotCalculator.Calculate(
      now, *alliance == frc::DriverStation::Alliance::kRed);
  double val = std::clamp(solution.flywheelSpeed, 0.0, 1500.0);
  m_flywheel->SetRPM(units::revolutions_per_minute_t{val});
}

void ShotCalculatedSpinUpCommand::End(bool interrupted) {
  // Keep flywheel hot; just release feeder/floor.
  m_feeder->Stop();
  m_floor->Stop();
}

bool ShotCalculatedSpinUpCommand::IsFinished() { return false; }
