// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/floor/FloorSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "utils/TunableDouble.h"

class SimpleShootCommand
    : public frc2::CommandHelper<frc2::Command, SimpleShootCommand> {
public:
  SimpleShootCommand(FlywheelSubsystem *flywheel, FeederSubsystem *feeder,
                     FloorSubsystem *floor, HoodSubsystem *hood,
                     IntakeBottomRollerSubsystem *bottomRoller,
                     IntakeDeployerSubsystem *deployer,
                     units::revolutions_per_minute_t flywheelRPM,
                     units::turns_per_second_t kickerRPS,
                     units::degree_t hoodAngle);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  FlywheelSubsystem *m_flywheel;
  FeederSubsystem *m_feeder;
  FloorSubsystem *m_floor;
  HoodSubsystem *m_hood;
  IntakeBottomRollerSubsystem *m_bottomRoller;
  IntakeDeployerSubsystem *m_deployer;

  units::revolutions_per_minute_t m_flywheelRPM;
  units::turns_per_second_t m_kickerRPS;
  units::degree_t m_hoodAngle;
  units::volt_t m_floorVolts{0_V};
  units::volt_t m_bottomVolts{0_V};

  units::second_t m_pulseStartTime{0_s};
  bool m_deployerExtended{false};

  static constexpr units::volt_t kFloorVoltage = 10_V;
  static constexpr units::volt_t kBottomVoltage = 10_V;
  TunableDouble m_tunableFlywheelRPM;
  TunableDouble m_tunableKickerRPS;
  TunableDouble m_tunableHoodAngle;
  TunableDouble m_tunableBottomVoltage;
  TunableDouble m_tunableFloorVoltage;
};
