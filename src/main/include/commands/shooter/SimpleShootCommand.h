// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "subsystem/floorroller/FloorRollerSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/kicker/KickerSubsystem.h"

class SimpleShootCommand
    : public frc2::CommandHelper<frc2::Command, SimpleShootCommand> {
public:
  SimpleShootCommand(FlywheelSubsystem *flywheel, KickerSubsystem *kicker,
                     FloorRollerSubsystem *floorRoller, HoodSubsystem *hood,
                     units::revolutions_per_minute_t flywheelRPM,
                     units::turns_per_second_t kickerRPS,
                     units::degree_t hoodAngle);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  FlywheelSubsystem *m_flywheel;
  KickerSubsystem *m_kicker;
  FloorRollerSubsystem *m_floorRoller;
  HoodSubsystem *m_hood;

  units::revolutions_per_minute_t m_flywheelRPM;
  units::turns_per_second_t m_kickerRPS;
  units::degree_t m_hoodAngle;

  static constexpr units::volt_t kFloorRollerVoltage = 10_V;
};
