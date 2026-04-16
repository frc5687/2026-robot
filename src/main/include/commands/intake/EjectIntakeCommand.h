// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/voltage.h>

#include "subsystem/floor/FloorSubsystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"

class EjectIntakeCommand
    : public frc2::CommandHelper<frc2::Command, EjectIntakeCommand> {
public:
  EjectIntakeCommand(IntakeDeployerSubsystem *deployer,
                     IntakeTopRollerSubsystem *topRoller,
                     IntakeBottomRollerSubsystem *bottomRoller,
                     FloorSubsystem *floor);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  InterruptionBehavior GetInterruptionBehavior() const override {
    return InterruptionBehavior::kCancelIncoming;
  }

private:
  IntakeDeployerSubsystem *m_deployer;
  IntakeTopRollerSubsystem *m_topRoller;
  IntakeBottomRollerSubsystem *m_bottomRoller;
  FloorSubsystem *m_floor;

  static constexpr units::volt_t kTopRollerVoltage = -6_V;
  static constexpr units::volt_t kBottomRollerVoltage = -6_V;
  static constexpr units::volt_t kFloorVoltage = -6_V;
};
