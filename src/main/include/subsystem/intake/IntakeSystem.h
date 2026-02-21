// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"

class IntakeSystem {
public:
  IntakeSystem(IntakeDeployerSubsystem &deployer,
               IntakeTopRollerSubsystem &topRoller,
               IntakeBottomRollerSubsystem &bottomRoller);

  void Deploy();
  void Retract();

  void RunRollers(units::volt_t topVoltage, units::volt_t bottomVoltage);
  void StopRollers();

  void StartIntaking(units::volt_t topVoltage, units::volt_t bottomVoltage);
  void StopIntaking();

  bool IsDeployed() const;
  bool IsRetracted() const;

private:
  IntakeDeployerSubsystem &m_deployer;
  IntakeTopRollerSubsystem &m_topRoller;
  IntakeBottomRollerSubsystem &m_bottomRoller;
};
