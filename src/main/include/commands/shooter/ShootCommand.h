// Team 5687 2026

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include <functional>
#include <numbers>

#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/kicker/KickerSubsystem.h"
#include "subsystem/shooter/ShotCalculator.h"

class ShootCommand : public frc2::CommandHelper<frc2::Command, ShootCommand> {
public:
  ShootCommand(DriveSubsystem *drive, FlywheelSubsystem *flywheel,
               HoodSubsystem *hood, IntakeBottomRollerSubsystem *bottomRoller,
               FeederSubsystem *feeder, KickerSubsystem *kicker,
               IntakeDeployerSubsystem *deployer,
               std::function<double()> throttle,
               std::function<double()> strafe);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  DriveSubsystem *m_drive;
  FlywheelSubsystem *m_flywheel;
  HoodSubsystem *m_hood;
  IntakeBottomRollerSubsystem *m_bottomRoller;
  FeederSubsystem *m_feeder;
  KickerSubsystem *m_kicker;
  IntakeDeployerSubsystem *m_deployer;

  std::function<double()> m_throttle;
  std::function<double()> m_strafe;

  ShotCalculator m_shotCalculator;

  frc::PIDController m_headingController{5.0, 0.0, 0.0};

  units::second_t m_pulseStartTime{0_s};
  bool m_deployerExtended{false};

  static constexpr units::volt_t kFeedVoltage = 10_V;
  static constexpr units::volt_t kBottomVoltage = 10_V;
  static constexpr units::turns_per_second_t kKickerRPS = 60_tps;
  static constexpr double kDeadband = 0.1;

  double ApplyDeadband(double value, double deadband);
};
