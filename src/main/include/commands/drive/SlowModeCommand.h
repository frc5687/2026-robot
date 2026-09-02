// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/voltage.h>

#include "subsystem/drive/DriveSubsystem.h"

class SlowModeCommand
    : public frc2::CommandHelper<frc2::Command, SlowModeCommand> {
public:
  SlowModeCommand(DriveSubsystem *drive);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  DriveSubsystem *m_drive;
};
