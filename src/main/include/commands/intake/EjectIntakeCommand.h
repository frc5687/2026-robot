// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/voltage.h>

#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"

class EjectIntakeCommand
    : public frc2::CommandHelper<frc2::Command, EjectIntakeCommand> {
public:
  EjectIntakeCommand(IntakeDeployerSubsystem *deployer,
                     IntakeTopRollerSubsystem *topRoller,
                     IntakeBottomRollerSubsystem *bottomRoller);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  IntakeDeployerSubsystem *m_deployer;
  IntakeTopRollerSubsystem *m_topRoller;
  IntakeBottomRollerSubsystem *m_bottomRoller;

  static constexpr units::volt_t kTopRollerVoltage = -3_V;
  static constexpr units::volt_t kBottomRollerVoltage = -3_V;
  static constexpr units::volt_t kFeederVoltage = -5_V;
};
