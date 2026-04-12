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
#include "utils/Utils.h"

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

  // Floor
  static constexpr units::volt_t kFloorVoltage = 12_V;
  // Feeder
  static constexpr units::turns_per_second_t kFeederRPS = 80_tps;
  // Top Roller
  static constexpr units::volt_t kTopVoltage = 6_V;
  // Bottom Roller
  static constexpr units::volt_t kBottomVoltage = 6_V;
  // Deployer pulse timing
  static constexpr units::second_t kDeployerExtendDelay = 1.5_s;
  static constexpr units::second_t kPulseRetractDuration = 0.2_s;
  static constexpr units::second_t kFinalRetractDuration = 0.6_s;
  static constexpr int kPulseCount = 1;

  static constexpr double kDeadband = 0.1;

  bool m_shootSequenceActive{false};
  units::second_t m_shootSequenceStartTime{0_s};

  // Deployer pulse state
  bool m_pulsingStarted{false};
  bool m_pulseRetracted{false};
  bool m_finalRetractStarted{false};
  int m_pulsesCompleted{0};
  units::second_t m_pulseStartTime{0_s};

  units::second_t m_burstTime{0.2_s};
  double m_burstOffset{200};
};
