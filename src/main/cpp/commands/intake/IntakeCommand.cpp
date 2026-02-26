// Team 5687 2026

#include "commands/intake/IntakeCommand.h"

IntakeCommand::IntakeCommand(IntakeDeployerSubsystem *deployer,
                             IntakeTopRollerSubsystem *topRoller,
                             IntakeBottomRollerSubsystem *bottomRoller)
    : m_deployer(deployer), m_topRoller(topRoller),
      m_bottomRoller(bottomRoller) {
  AddRequirements({deployer, topRoller, bottomRoller});
  SetName("IntakeCommand");
}

void IntakeCommand::Initialize() { m_deployer->Deploy(); }

void IntakeCommand::Execute() {
  m_deployer->Deploy();
  m_topRoller->SetVoltage(kTopRollerVoltage);
  m_bottomRoller->SetVoltage(kBottomRollerVoltage);
}

void IntakeCommand::End(bool interrupted) {
  // m_deployer->RetractMid();
  m_topRoller->Stop();
  m_bottomRoller->Stop();
}

bool IntakeCommand::IsFinished() { return false; }
