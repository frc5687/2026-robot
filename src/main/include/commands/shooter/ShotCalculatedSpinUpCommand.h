// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/shooter/ShotCalculator.h"

class ShotCalculatedSpinUpCommand
    : public frc2::CommandHelper<frc2::Command, ShotCalculatedSpinUpCommand> {
public:
  ShotCalculatedSpinUpCommand(FlywheelSubsystem *flywheel, FeederSubsystem *feeder);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  FlywheelSubsystem *m_flywheel;
  FeederSubsystem *m_feeder;
  ShotCalculator m_shotCalculator;

  units::turn_t m_reverseAmount = 4_tr;
  units::second_t m_startTime = 0_s;
};
