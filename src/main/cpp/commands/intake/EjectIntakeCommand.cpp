// Team 5687 2026

#include "commands/intake/EjectIntakeCommand.h"

EjectIntakeCommand::EjectIntakeCommand(IntakeDeployerSubsystem *deployer,
                             IntakeTopRollerSubsystem *topRoller,
                             IntakeBottomRollerSubsystem *bottomRoller)
    : m_deployer(deployer), m_topRoller(topRoller),
      m_bottomRoller(bottomRoller) {
  AddRequirements({deployer, topRoller, bottomRoller});
  SetName("IntakeCommand");
}

void EjectIntakeCommand::Initialize() {
  m_deployer->Deploy();
}

void EjectIntakeCommand::Execute() {
  m_deployer->Deploy();
  m_topRoller->SetVoltage(kTopRollerVoltage);
  m_bottomRoller->SetVoltage(kBottomRollerVoltage);
}

void EjectIntakeCommand::End(bool interrupted) {
  // m_deployer->RetractMid();
  m_topRoller->Stop();
  m_bottomRoller->Stop();
}

bool EjectIntakeCommand::IsFinished() { return false; }
