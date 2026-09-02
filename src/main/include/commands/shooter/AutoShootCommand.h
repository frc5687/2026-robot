// Team 5687 2026

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include "subsystem/drive/SwerveDriveConstants.h"
#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/floor/FloorSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"
#include "subsystem/shooter/ShotCalculator.h"

class AutoShootCommand
    : public frc2::CommandHelper<frc2::Command, AutoShootCommand> {
public:
  AutoShootCommand(FlywheelSubsystem *flywheel, HoodSubsystem *hood,
                   FeederSubsystem *feeder, FloorSubsystem *floor,
                   IntakeTopRollerSubsystem *topRoller,
                   IntakeBottomRollerSubsystem *bottomRoller,
                   IntakeDeployerSubsystem *deployer);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  FlywheelSubsystem *m_flywheel;
  HoodSubsystem *m_hood;
  FeederSubsystem *m_feeder;
  FloorSubsystem *m_floor;
  IntakeTopRollerSubsystem *m_topRoller;
  IntakeBottomRollerSubsystem *m_bottomRoller;
  IntakeDeployerSubsystem *m_deployer;

  ShotCalculator m_shotCalculator;

  frc::PIDController m_headingController{
      Constants::SwerveDrive::Shooting::kAimkP, 0.0, 0.0};

  units::radians_per_second_t m_rotationFeedback{0_rad_per_s};

  units::second_t m_pulseStartTime{0_s};
  bool m_deployerExtended{false};
  bool m_clearanceComplete{false};
  units::second_t m_clearanceStartTime{0_s};
  bool m_hasFedFuel{false};

  static constexpr units::ampere_t kFloorCurrent = 40_A;
  static constexpr units::volt_t kFloorVoltage = 12_V;
  static constexpr units::volt_t kBackoffFloorVoltage = -2.0_V;
  static constexpr units::volt_t kPreclearFlywheelReverseVoltage = -1.5_V;
  static constexpr units::volt_t kTopVoltage = 10_V;
  static constexpr units::volt_t kBottomVoltage = 12_V;
  static constexpr units::turns_per_second_t kKickerRPS = 75_tps;
  static constexpr units::volt_t kFeederVoltage = 12_V;
  static constexpr units::second_t kPulseExtendDuration = 0.3_s;
  static constexpr units::second_t kPulseRetractDuration = 0.25_s;
};
