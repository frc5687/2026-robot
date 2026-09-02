// Team 5687 2026

#include "commands/intake/EjectIntakeCommand.h"

EjectIntakeCommand::EjectIntakeCommand(
    IntakeDeployerSubsystem *deployer, IntakeTopRollerSubsystem *topRoller,
    IntakeBottomRollerSubsystem *bottomRoller, FloorSubsystem *floor)
    : m_deployer(deployer), m_topRoller(topRoller),
      m_bottomRoller(bottomRoller), m_floor(floor) {
  AddRequirements({deployer, topRoller, bottomRoller, floor});
  SetName("EjectIntakeCommand");
}

void EjectIntakeCommand::Initialize() { m_deployer->Deploy(); }

void EjectIntakeCommand::Execute() {
  m_deployer->Deploy();
  m_topRoller->SetVoltage(kTopRollerVoltage);
  m_bottomRoller->SetVoltage(kBottomRollerVoltage);
  m_floor->SetVoltage(kFloorVoltage);
}

void EjectIntakeCommand::End(bool interrupted) {
  // m_deployer->RetractMid();
  m_floor->Stop();
  m_topRoller->Stop();
  m_bottomRoller->Stop();
}

bool EjectIntakeCommand::IsFinished() { return false; }
