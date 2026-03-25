// Team 5687 2026

#include "commands/intake/EjectIntakeCommand.h"

#include "subsystem/feeder/FeederSubsystem.h"

EjectIntakeCommand::EjectIntakeCommand(
    IntakeDeployerSubsystem *deployer, IntakeTopRollerSubsystem *topRoller,
    IntakeBottomRollerSubsystem *bottomRoller, FeederSubsystem *feeder)
    : m_deployer(deployer), m_topRoller(topRoller),
      m_bottomRoller(bottomRoller), m_feeder(feeder) {
  AddRequirements({deployer, topRoller, bottomRoller, feeder});
  SetName("IntakeCommand");
}

void EjectIntakeCommand::Initialize() { m_deployer->Deploy(); }

void EjectIntakeCommand::Execute() {
  m_deployer->Deploy();
  m_topRoller->SetVoltage(kTopRollerVoltage);
  m_bottomRoller->SetVoltage(kBottomRollerVoltage);
  m_feeder->SetVoltage(kFeederVoltage);
}

void EjectIntakeCommand::End(bool interrupted) {
  // m_deployer->RetractMid();
  m_feeder->Stop();
  m_topRoller->Stop();
  m_bottomRoller->Stop();
}

bool EjectIntakeCommand::IsFinished() { return false; }
