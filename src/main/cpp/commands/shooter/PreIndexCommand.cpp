// Team 5687 2026

#include "commands/shooter/PreIndexCommand.h"

#include <frc/Timer.h>

#include "subsystem/feeder/FeederConstants.h"

PreIndexCommand::PreIndexCommand(FlywheelSubsystem *flywheel,
                                 FeederSubsystem *feeder, FloorSubsystem *floor)
    : m_flywheel(flywheel), m_feeder(feeder), m_floor(floor) {
  AddRequirements({flywheel, feeder, floor});
  SetName("PreIndexCommand");
}

void PreIndexCommand::Initialize() {
  m_startTime = frc::Timer::GetFPGATimestamp();
  m_feeder->SetIndexingActive(false);
  m_clearanceComplete = !m_feeder->NeedsIndexing();
  if (!m_clearanceComplete) {
    m_feeder->BeginClearance();
  }
}

void PreIndexCommand::Execute() {
  m_flywheel->SetRPM(0_rpm);

  if (m_clearanceComplete) {
    m_feeder->Stop();
    m_floor->Stop();
    return;
  }

  auto now = frc::Timer::GetFPGATimestamp();
  if (m_feeder->IsCleared() ||
      now - m_startTime >= Constants::Feeder::kClearanceTimeout) {
    m_clearanceComplete = true;
    m_feeder->SetIndexed();
    m_feeder->Stop();
    m_floor->Stop();
    return;
  }

  m_floor->SetVoltage(kBackoffFloorVoltage);
}

void PreIndexCommand::End(bool interrupted) {
  m_flywheel->SetRPM(0_rpm);
  m_feeder->Stop();
  m_floor->Stop();
}

bool PreIndexCommand::IsFinished() { return m_clearanceComplete; }
