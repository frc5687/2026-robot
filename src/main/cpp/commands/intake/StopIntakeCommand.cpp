// Team 5687 2026

#include "commands/intake/StopIntakeCommand.h"

StopIntakeCommand::StopIntakeCommand(IntakeDeployerSubsystem *deployer,
                             IntakeTopRollerSubsystem *topRoller,
                             IntakeBottomRollerSubsystem *bottomRoller)
    : m_deployer(deployer), m_topRoller(topRoller),
      m_bottomRoller(bottomRoller) {
  AddRequirements({deployer, topRoller, bottomRoller});
  SetName("StopIntakeCommand");
}

void StopIntakeCommand::Initialize() { m_deployer->Deploy(); }

void StopIntakeCommand::Execute() {
  m_deployer->Deploy();
  m_topRoller->SetVoltage(kTopRollerVoltage);
  m_bottomRoller->SetVoltage(kBottomRollerVoltage);
}

void StopIntakeCommand::End(bool interrupted) {
  // m_deployer->RetractMid();
  m_topRoller->Stop();
  m_bottomRoller->Stop();
}

bool StopIntakeCommand::IsFinished() { return false; }
