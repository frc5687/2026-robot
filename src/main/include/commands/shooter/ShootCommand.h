// Team 5687 2026

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include <functional>

#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/drive/SwerveDriveConstants.h"
#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/floor/FloorSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"
#include "subsystem/shooter/ShotCalculator.h"
#include "utils/TunableDouble.h"

class ShootCommand : public frc2::CommandHelper<frc2::Command, ShootCommand> {
public:
  ShootCommand(DriveSubsystem *drive, FlywheelSubsystem *flywheel,
               HoodSubsystem *hood, IntakeTopRollerSubsystem *topRoller,
               IntakeBottomRollerSubsystem *bottomRoller,
               FeederSubsystem *feeder, FloorSubsystem *floor,
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
  IntakeTopRollerSubsystem *m_topRoller;
  IntakeBottomRollerSubsystem *m_bottomRoller;
  FeederSubsystem *m_feeder;
  FloorSubsystem *m_floor;
  IntakeDeployerSubsystem *m_deployer;

  std::function<double()> m_throttle;
  std::function<double()> m_strafe;

  ShotCalculator m_shotCalculator;

  frc::PIDController m_headingController{
      Constants::SwerveDrive::Shooting::kAimkP, 0.0,
      Constants::SwerveDrive::Shooting::kAimkD};

  static constexpr units::ampere_t kFloorCurrent = 30_A;
  static constexpr units::ampere_t kFeederCurrent = 100_A;
  static constexpr units::volt_t kTopVoltage = 6_V;
  static constexpr units::volt_t kBottomVoltage = 6_V;
  static constexpr units::second_t kDeployerRetractDelay = 1.5_s;
  static constexpr double kDeadband = 0.1;

  bool m_shootSequenceActive{false};
  bool m_hasRetractedDeployer{false};
  units::second_t m_shootBurstStartTime{0_s};

  double ApplyDeadband(double value, double deadband);
};
