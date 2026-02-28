// Team 5687 2026

#include "commands/shooter/ShootCommand.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include "Constants.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"

ShootCommand::ShootCommand(DriveSubsystem *drive, FlywheelSubsystem *flywheel,
                           HoodSubsystem *hood,
                           IntakeBottomRollerSubsystem *bottomRoller,
                           FeederSubsystem *feeder, KickerSubsystem *kicker,
                           IntakeDeployerSubsystem *deployer,
                           std::function<double()> throttle,
                           std::function<double()> strafe)
    : m_drive(drive), m_flywheel(flywheel), m_hood(hood),
      m_bottomRoller(bottomRoller), m_feeder(feeder), m_kicker(kicker),
      m_deployer(deployer), m_throttle(throttle), m_strafe(strafe) {
  AddRequirements({drive, flywheel, hood, feeder, kicker, deployer});
  SetName("ShootCommand");
  m_headingController.EnableContinuousInput(-std::numbers::pi,
                                            std::numbers::pi);
  m_headingController.SetTolerance(0.035); // ~2 deg
}

void ShootCommand::Initialize() {
  m_headingController.Reset();
  m_drive->SetMaxSpeeds(
      Constants::SwerveDrive::Shooting::kMaxSpeedsWhileShooting);
  m_deployer->RetractMid();
  m_deployerExtended = false;
  m_pulseStartTime = frc::Timer::GetFPGATimestamp();
}

void ShootCommand::Execute() {
  auto now = frc::Timer::GetFPGATimestamp();
  auto alliance = frc::DriverStation::GetAlliance();
  bool isRed = alliance == frc::DriverStation::Alliance::kRed;

  auto solution = m_shotCalculator.Calculate(now, isRed);

  m_flywheel->SetRPM(units::revolutions_per_minute_t{solution.flywheelSpeed});

  m_hood->SetPosition(units::radian_t{solution.hoodAngle});

  double throttle = ApplyDeadband(m_throttle(), kDeadband);
  double strafe = ApplyDeadband(m_strafe(), kDeadband);

  auto maxSpeeds = m_drive->GetMaxSpeeds();
  units::meters_per_second_t maxLinearSpeed = maxSpeeds.first;
  auto xVel = throttle * maxLinearSpeed;
  auto yVel = strafe * maxLinearSpeed;

  double rotOutput =
      m_headingController.Calculate(m_drive->GetOdometryThread()
                                        ->GetEstimatedPose()
                                        .Rotation()
                                        .Radians()
                                        .value(),
                                    solution.driveAngle.Radians().value());
  rotOutput = -rotOutput;

  rotOutput =
      std::clamp(rotOutput, -Constants::SwerveDrive::kMaxAngularSpeed.value(),
                 Constants::SwerveDrive::kMaxAngularSpeed.value());

  m_drive->DriveFieldRelative(
      frc::ChassisSpeeds{xVel, yVel, units::radians_per_second_t{rotOutput}});

  auto elapsed = now - m_pulseStartTime;

  if (m_deployerExtended) {
    if (elapsed >= Constants::IntakeDeployer::kPulseExtendDuration) {
      m_deployer->RetractMid();
      m_deployerExtended = false;
      m_pulseStartTime = now;
    }
  } else {
    if (elapsed >= Constants::IntakeDeployer::kPulseRetractDuration) {
      m_deployer->Deploy();
      m_deployerExtended = true;
      m_pulseStartTime = now;
    }
  }

  if (solution.ready) {
    m_feeder->SetVoltage(kFeedVoltage);
    m_bottomRoller->SetVoltage(kBottomVoltage);
    m_kicker->SetVelocity(kKickerRPS);
  } else {
    m_feeder->Stop();
    m_kicker->Stop();
  }
}

void ShootCommand::End(bool interrupted) {
  m_flywheel->SetRPM(0_rpm);
  m_hood->SetPosition(0_rad);
  m_bottomRoller->Stop();
  m_feeder->Stop();
  m_kicker->Stop();
  m_deployer->RetractMid();
  m_drive->SetMaxSpeeds(Constants::SwerveDrive::kMaxLinearSpeed);
  m_drive->Stop();
}

bool ShootCommand::IsFinished() { return false; }

double ShootCommand::ApplyDeadband(double value, double deadband) {
  if (std::abs(value) < deadband)
    return 0.0;
  return (value - std::copysign(deadband, value)) / (1.0 - deadband);
}
