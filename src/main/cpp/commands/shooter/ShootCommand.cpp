// Team 5687 2026

#include "commands/shooter/ShootCommand.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include "utils/Logger.h"

ShootCommand::ShootCommand(DriveSubsystem *drive, FlywheelSubsystem *flywheel,
                           HoodSubsystem *hood,
                           IntakeTopRollerSubsystem *topRoller,
                           IntakeBottomRollerSubsystem *bottomRoller,
                           FeederSubsystem *feeder, FloorSubsystem *floor,
                           IntakeDeployerSubsystem *deployer,
                           std::function<double()> throttle,
                           std::function<double()> strafe)
    : m_drive(drive), m_flywheel(flywheel), m_hood(hood),
      m_topRoller(topRoller), m_bottomRoller(bottomRoller), m_feeder(feeder),
      m_floor(floor), m_deployer(deployer), m_throttle(throttle),
      m_strafe(strafe) {
  AddRequirements({drive, flywheel, hood, feeder, floor, deployer});
  SetName("ShootCommand");
  m_headingController.EnableContinuousInput(-std::numbers::pi,
                                            std::numbers::pi);
  m_headingController.SetTolerance(0.035); // ~2 deg
}

void ShootCommand::Initialize() {
  m_headingController.Reset();
  m_drive->SetMaxSpeeds(
      Constants::SwerveDrive::Shooting::kMaxSpeedsWhileShooting);
  m_hasRetractedDeployer = false;
  m_shootSequenceActive = false;
  m_shootBurstStartTime = 0_s;
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
  rotOutput =
      std::clamp(rotOutput, -Constants::SwerveDrive::kMaxAngularSpeed.value(),
                 Constants::SwerveDrive::kMaxAngularSpeed.value());

  m_drive->DriveFieldRelative(
      frc::ChassisSpeeds{xVel, yVel, units::radians_per_second_t{rotOutput}});

  if (!m_shootSequenceActive && solution.ready) {
    m_shootSequenceActive = true;
    m_shootBurstStartTime = now;
    m_hasRetractedDeployer = false;
  }

  if (m_shootSequenceActive) {
    if (!m_hasRetractedDeployer &&
        now - m_shootBurstStartTime >= kDeployerRetractDelay) {
      m_deployer->RetractMid();
      m_hasRetractedDeployer = true;
    }
  }

  auto speeds = m_drive->GetFieldRelativeSpeeds();
  if (units::math::abs(speeds.vx) > 0.25_mps ||
      units::math::abs(speeds.vy) > 0.25_mps) {
    m_deployer->Deploy(); // FIXME: change to restarting timer
  }

  auto &log = Logger::Instance();
  log.Log("ShootCommand/ShootSequenceActive", m_shootSequenceActive);
  log.Log("ShootCommand/SolutionReady", solution.ready);
  log.Log("ShootCommand/FeederCurrent/Requested", kFeederCurrent.value());
  log.Log("ShootCommand/FloorCurrent/Requested", kFloorCurrent.value());

  if (m_shootSequenceActive) {
    //m_floor->SetCurrent(kFloorCurrent);
    m_floor->SetVoltage(4_V);
    //m_feeder->SetVoltage(12_V);
    m_feeder->SetVelocity(80_tps);
    //m_feeder->SetCurrent(kFeederCurrent);
    m_topRoller->SetVoltage(kTopVoltage);
    m_bottomRoller->SetVoltage(kBottomVoltage);
  } else {
    m_floor->Stop();
    m_feeder->Stop();
  }
}

void ShootCommand::End(bool interrupted) {
  m_flywheel->SetRPM(0_rpm);
  m_hood->SetPosition(0_rad);
  m_topRoller->Stop();
  m_bottomRoller->Stop();
  m_feeder->Stop();
  m_floor->Stop();
  m_deployer->RetractMid();
  m_feeder->ClearIndexed();
  m_hasRetractedDeployer = false;
  m_shootSequenceActive = false;
  m_shootBurstStartTime = 0_s;
  m_drive->SetMaxSpeeds(Constants::SwerveDrive::kMaxLinearSpeed);
  m_drive->Stop();
}

bool ShootCommand::IsFinished() { return false; }

double ShootCommand::ApplyDeadband(double value, double deadband) {
  if (std::abs(value) < deadband)
    return 0.0;
  return (value - std::copysign(deadband, value)) / (1.0 - deadband);
}
