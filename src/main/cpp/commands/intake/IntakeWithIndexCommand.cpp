// Team 5687 2026

#include "commands/intake/IntakeWithIndexCommand.h"

#include <frc/Timer.h>
#include <units/math.h>

IntakeWithIndexCommand::IntakeWithIndexCommand(
    IntakeDeployerSubsystem *deployer, IntakeTopRollerSubsystem *topRoller,
    IntakeBottomRollerSubsystem *bottomRoller, FloorSubsystem *floor,
    FeederSubsystem *feeder, FlywheelSubsystem *flywheel)
    : m_deployer(deployer), m_topRoller(topRoller),
      m_bottomRoller(bottomRoller), m_floor(floor), m_feeder(feeder),
      m_flywheel(flywheel) {
  AddRequirements({deployer, topRoller, bottomRoller, floor, feeder});
  SetName("IntakeWithIndexCommand");
}

void IntakeWithIndexCommand::Initialize() {
  m_deployer->Deploy();
  m_state = State::Intaking;
  m_stateStartTime = frc::Timer::GetFPGATimestamp();
  m_usingFlywheel = false;
  m_indexStartPosition = 0_tr;
}

void IntakeWithIndexCommand::Execute() {
  auto now = frc::Timer::GetFPGATimestamp();
  auto elapsed = now - m_stateStartTime;

  m_deployer->Deploy();
  m_topRoller->SetVoltage(kTopRollerVoltage);
  m_bottomRoller->SetVoltage(kBottomRollerVoltage);
  m_floor->SetVoltage(kFloorIntakeVoltage);

  switch (m_state) {
  case State::Intaking:
    if (m_feeder->NeedsIndexing() &&
        elapsed >= kIntakeBeforeIndexDuration && IsFlywheelIdle()) {
      m_state = State::Indexing;
      m_indexStartPosition = m_feeder->GetPosition();
      m_usingFlywheel = true;
    }
    break;

  case State::Indexing:
    if (!IsFlywheelIdle()) {
      m_flywheel->SetVoltage(0_V);
      m_feeder->Stop();
      m_usingFlywheel = false;
      m_state = State::Intaking;
      m_stateStartTime = now;
      break;
    }

    m_feeder->SetPosition(m_indexStartPosition + kFeederForwardDistance);
    m_flywheel->SetVoltage(kFlywheelReverseVoltage);

    if (m_feeder->GetPosition() >=
        m_indexStartPosition + kFeederForwardDistance - kPositionTolerance) {
      m_state = State::Reversing;
      m_flywheel->SetVoltage(0_V);
    }
    break;

  case State::Reversing: {
    if (!IsFlywheelIdle()) {
      m_feeder->Stop();
      m_usingFlywheel = false;
      m_feeder->SetIndexed();
      m_state = State::Intaking;
      m_stateStartTime = now;
      break;
    }

    auto reverseTarget = m_indexStartPosition + kFeederForwardDistance -
                         kFeederReverseDistance;
    m_feeder->SetPosition(reverseTarget);
    m_flywheel->SetVoltage(0_V);

    if (m_feeder->GetPosition() <= reverseTarget + kPositionTolerance) {
      m_usingFlywheel = false;
      m_feeder->Stop();
      m_feeder->SetIndexed();
      m_state = State::Intaking;
      m_stateStartTime = now;
    }
    break;
  }
  }
}

void IntakeWithIndexCommand::End(bool interrupted) {
  m_topRoller->Stop();
  m_bottomRoller->Stop();
  m_floor->Stop();
  m_feeder->Stop();
  if (m_usingFlywheel) {
    m_flywheel->SetVoltage(0_V);
    m_usingFlywheel = false;
  }
}

bool IntakeWithIndexCommand::IsFinished() { return false; }

bool IntakeWithIndexCommand::IsFlywheelIdle() const {
  return m_flywheel->GetFilteredRPM() < kFlywheelIdleThreshold;
}
