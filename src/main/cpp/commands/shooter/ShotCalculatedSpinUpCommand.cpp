// Team 5687 2026

#include "commands/shooter/ShotCalculatedSpinUpCommand.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>

#include <algorithm>

#include "utils/MatchTracker.h"

ShotCalculatedSpinUpCommand::ShotCalculatedSpinUpCommand(
    FlywheelSubsystem *flywheel)
    : m_flywheel(flywheel) {
  AddRequirements(flywheel);
  SetName("ShotCalculatedSpinUpCommand");
}

void ShotCalculatedSpinUpCommand::Initialize() {}

void ShotCalculatedSpinUpCommand::Execute() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (!alliance.has_value()) {
    m_flywheel->SetRPM(0_rpm);
    return;
  }

  // if (!MatchTracker::Instance().ShouldPrespinForAlliance(*alliance)) {
  //   m_flywheel->SetRPM(0_rpm);
  //   return;
  // }

  auto solution = m_shotCalculator.Calculate(
      frc::Timer::GetFPGATimestamp(),
      *alliance == frc::DriverStation::Alliance::kRed);
  double val = std::clamp(solution.flywheelSpeed, 0.0, 1200.0);
  m_flywheel->SetRPM(units::revolutions_per_minute_t{val});
}

void ShotCalculatedSpinUpCommand::End(bool interrupted) {
  // Do not set to 0
  // m_flywheel->SetRPM(0_rpm);
}

bool ShotCalculatedSpinUpCommand::IsFinished() { return false; }
