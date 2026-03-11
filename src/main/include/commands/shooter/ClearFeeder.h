// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/voltage.h>

#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/kicker/KickerSubsystem.h"

class ClearFeeder 
    : public frc2::CommandHelper<frc2::Command, ClearFeeder> {
public:
  ClearFeeder(KickerSubsystem *kicker,
                     FeederSubsystem* feeder
                    );

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  KickerSubsystem *m_kicker;
  FeederSubsystem *m_feeder;

  static constexpr units::volt_t kFeederVoltage = -6_V;
  static constexpr units::volt_t kKickerVoltage = -6_V;
};
