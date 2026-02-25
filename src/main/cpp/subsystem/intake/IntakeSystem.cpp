// Team 5687 2026

#include "subsystem/intake/IntakeSystem.h"

#include "Constants.h"

IntakeSystem::IntakeSystem(IntakeDeployerSubsystem &deployer,
                           IntakeTopRollerSubsystem &topRoller,
                           IntakeBottomRollerSubsystem &bottomRoller)
    : m_deployer(deployer), m_topRoller(topRoller),
      m_bottomRoller(bottomRoller) {}

void IntakeSystem::Deploy() { m_deployer.Deploy(); }

void IntakeSystem::Retract() { m_deployer.Retract(); }

void IntakeSystem::RunRollers(units::volt_t topVoltage,
                              units::volt_t bottomVoltage) {
  m_topRoller.SetVoltage(topVoltage);
  m_bottomRoller.SetVoltage(bottomVoltage);
}

void IntakeSystem::StopRollers() {
  m_topRoller.Stop();
  m_bottomRoller.Stop();
}

void IntakeSystem::StartIntaking(units::volt_t topVoltage,
                                 units::volt_t bottomVoltage) {
  Deploy();
  RunRollers(topVoltage, bottomVoltage);
}

void IntakeSystem::StopIntaking() {
  Retract();
  StopRollers();
}

bool IntakeSystem::IsDeployed() const { return m_deployer.IsDeployed(); }

bool IntakeSystem::IsRetracted() const { return m_deployer.IsRetracted(); }
