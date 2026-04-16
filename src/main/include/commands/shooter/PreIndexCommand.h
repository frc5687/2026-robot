// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/time.h>
#include <units/voltage.h>

#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/floor/FloorSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"

class PreIndexCommand
    : public frc2::CommandHelper<frc2::Command, PreIndexCommand> {
public:
  PreIndexCommand(FlywheelSubsystem *flywheel, FeederSubsystem *feeder,
                  FloorSubsystem *floor);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  FlywheelSubsystem *m_flywheel;
  FeederSubsystem *m_feeder;
  FloorSubsystem *m_floor;

  units::second_t m_startTime{0_s};
  bool m_clearanceComplete{false};

  static constexpr units::volt_t kBackoffFloorVoltage = -2.0_V;
};
