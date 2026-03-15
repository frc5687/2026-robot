// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/voltage.h>

#include "subsystem/feeder/FeederSubsystem.h"

class ClearFeeder 
    : public frc2::CommandHelper<frc2::Command, ClearFeeder> {
public:
  ClearFeeder(FeederSubsystem* feeder);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  FeederSubsystem *m_feeder;

  static constexpr units::volt_t kFeederVoltage = -6_V;
};
