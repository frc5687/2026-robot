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
  AddRequirements({deployer, topRoller, bottomRoller, floor});
  SetName("IntakeWithIndexCommand");
}

void IntakeWithIndexCommand::Initialize() {
  m_deployer->Deploy();
}

void IntakeWithIndexCommand::Execute() {
 

  m_deployer->Deploy();
  m_topRoller->SetVoltage(kTopRollerVoltage);
  m_bottomRoller->SetVoltage(kBottomRollerVoltage);
  m_floor->SetVoltage(kFloorIntakeVoltage);
  m_flywheel->SetVoltage(kFlywheelReverseVoltage);
  if(IsFlywheelIdle()){
  m_feeder->SetVoltage(kFeederVoltage);
  }else{
    m_feeder->SetVoltage(0_V);
    m_floor->SetVoltage(0_V);
  }
}

void IntakeWithIndexCommand::End(bool interrupted) {
  m_topRoller->Stop();
  m_bottomRoller->Stop();
  m_floor->Stop();
  m_feeder->Stop();
  m_flywheel->SetVoltage(0_V);
  m_feeder->SetPosition(m_feeder->GetPosition()-kFeederReverseDistance);
}

bool IntakeWithIndexCommand::IsFinished() { return false; }

bool IntakeWithIndexCommand::IsFlywheelIdle() const {
  return m_flywheel->GetFilteredRPM() < kFlywheelIdleThreshold;
}
