// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "subsystem/flywheel/FlywheelSubsystem.h"

// Default flywheel reverse hardstop.
class FlywheelHardstopCommand
    : public frc2::CommandHelper<frc2::Command, FlywheelHardstopCommand> {
public:
  explicit FlywheelHardstopCommand(FlywheelSubsystem *flywheel);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  FlywheelSubsystem *m_flywheel;

  static constexpr units::volt_t kHardstopVoltage = -1.5_V;
  static constexpr units::revolutions_per_minute_t kIdleThreshold = 100_rpm;
};
