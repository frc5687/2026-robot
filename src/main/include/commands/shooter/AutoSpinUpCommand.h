// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include "subsystem/flywheel/FlywheelSubsystem.h"

class AutoSpinUpCommand
    : public frc2::CommandHelper<frc2::Command, AutoSpinUpCommand> {
public:
  AutoSpinUpCommand(FlywheelSubsystem *flywheel);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  FlywheelSubsystem *m_flywheel;
  
};
