// Team 5687 2026

#include "commands/shooter/IndexFeederCommand.h"

#include <frc/Timer.h>

IndexFeederCommand::IndexFeederCommand(FeederSubsystem *feeder,
                                       FloorSubsystem *floor,
                                       FlywheelSubsystem *flywheel)
    : m_feeder(feeder), m_floor(floor), m_flywheel(flywheel) {
  AddRequirements({m_feeder, m_floor, m_flywheel});
  SetName("IndexFeederCommand");
}

void IndexFeederCommand::Initialize() {
  m_state = State::Indexing;
  m_stateStartTime = frc::Timer::GetFPGATimestamp();
}

void IndexFeederCommand::Execute() {
  auto now = frc::Timer::GetFPGATimestamp();
  auto elapsed = now - m_stateStartTime;

  switch (m_state) {
  case State::Indexing:
    m_floor->SetVoltage(kFloorIndexVoltage);
    m_feeder->SetVoltage(kFloorIndexVoltage);
    m_flywheel->SetVoltage(kFlywheelReverseVoltage);

    if (elapsed >= kIndexDuration) {
      m_state = State::Reversing;
      m_stateStartTime = now;
    }
    break;

  case State::Reversing:
    //m_floor->SetVoltage(kFloorReverseVoltage);
    m_feeder->SetVoltage(kFloorIndexVoltage);
    m_flywheel->SetVoltage(0_V);

    if (elapsed >= kReverseDuration) {
      m_state = State::Done;
    }
    break;

  case State::Done:
    m_floor->Stop();
    break;
  }
}

void IndexFeederCommand::End(bool interrupted) {
  m_floor->Stop();
  m_feeder->Stop();
  m_flywheel->SetRPM(0_rpm);
}

bool IndexFeederCommand::IsFinished() { return m_state == State::Done; }
