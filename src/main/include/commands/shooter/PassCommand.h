// Team 5687 2026

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include "frc/controller/PIDController.h"
#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/feeder/FeederSubsystem.h"
#include "subsystem/floor/FloorSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"
#include "utils/TunableDouble.h"
#include "utils/Utils.h"

class PassCommand : public frc2::CommandHelper<frc2::Command, PassCommand> {
public:
  PassCommand(DriveSubsystem *drive, FlywheelSubsystem *flywheel,
              HoodSubsystem *hood, IntakeTopRollerSubsystem *topRoller,
              IntakeBottomRollerSubsystem *bottomRoller,
              FeederSubsystem *feeder, FloorSubsystem *floor,
              IntakeDeployerSubsystem *deployer,
              std::function<double()> throttle, std::function<double()> strafe);

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

  TunableDouble m_tunableRPM{"PassCommand", "FlywheelRPM", 1000.0};
  TunableDouble m_tunableHoodAngle{"PassCommand", "HoodAngleDeg", 20.0};

  std::function<double()> m_throttle;
  std::function<double()> m_strafe;

  frc::PIDController m_headingController{
      Constants::SwerveDrive::Shooting::kAimkP, 0.0,
      Constants::SwerveDrive::Shooting::kAimkD};
  // Floor
  static constexpr units::volt_t kFloorVoltage = 12_V;
  // Feeder
  static constexpr units::turns_per_second_t kFeederRPS = 80_tps;
  static constexpr units::volt_t kFeederVoltage = 12_V;
  static constexpr units::volt_t kBackoffFloorVoltage = -2.0_V;
  static constexpr units::volt_t kPreclearFlywheelReverseVoltage = -1.5_V;
  // Top Roller
  static constexpr units::volt_t kTopVoltage = 6_V;
  // Bottom Roller
  static constexpr units::volt_t kBottomVoltage = 10_V;
  // Deployer
  static constexpr units::second_t kDeployerExtendDelay = 1.5_s;
  static constexpr units::second_t kSlowRetractDuration = 1.0_s;

  bool m_shootSequenceActive{false};
  bool m_slowRetractStarted{false};
  bool m_clearanceComplete{false};
  units::second_t m_clearanceStartTime{0_s};
  units::second_t m_shootSequenceStartTime{0_s};
  bool m_hasFedFuel{false};
  static constexpr double kDeadband = 0.1;

  units::degree_t m_targetHeading{90};
};
