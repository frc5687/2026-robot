// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/voltage.h>

#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/floor/FloorSubsystem.h"

// Default feeder+floor indexing command.
class IndexCommand : public frc2::CommandHelper<frc2::Command, IndexCommand> {
public:
  IndexCommand(FeederSubsystem *feeder, FloorSubsystem *floor);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  FeederSubsystem *m_feeder;
  FloorSubsystem *m_floor;

  static constexpr units::volt_t kFeederVoltage = 1.0_V;
  static constexpr units::volt_t kFloorIntakeVoltage = 8_V;
  static constexpr units::volt_t kFloorIdleVoltage = 2.0_V;
};
