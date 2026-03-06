// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/shooter/ShotCalculator.h"

class ShotCalculatedSpinUpCommand
    : public frc2::CommandHelper<frc2::Command, ShotCalculatedSpinUpCommand> {
public:
  ShotCalculatedSpinUpCommand(FlywheelSubsystem *flywheel);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  FlywheelSubsystem *m_flywheel;
  ShotCalculator m_shotCalculator;
};
