// Team 5687 2026

#include "commands/shooter/PassCommand.h"

#include <frc/Timer.h>
#include <functional>
#include <units/angle.h>

#include "frc/DriverStation.h"
#include "subsystem/drive/DriveSubsystem.h"
#include "utils/Logger.h"

PassCommand::PassCommand(DriveSubsystem *drive,
    FlywheelSubsystem *flywheel, HoodSubsystem *hood,
    IntakeTopRollerSubsystem *topRoller,
    IntakeBottomRollerSubsystem *bottomRoller, FeederSubsystem *feeder,
    FloorSubsystem *floor, IntakeDeployerSubsystem *deployer,
    std::function<double()> throttle, std::function<double()> strafe)
    : m_drive(drive), m_flywheel(flywheel), m_hood(hood), m_topRoller(topRoller),
      m_bottomRoller(bottomRoller), m_feeder(feeder), m_floor(floor),
      m_deployer(deployer), m_throttle(throttle),
      m_strafe(strafe) {
  AddRequirements({drive, flywheel, hood, feeder, floor, deployer});
  SetName("PassCommand");
}

void PassCommand::Initialize() {
  m_shootSequenceActive = false;
  m_slowRetractStarted = false;
}

void PassCommand::Execute() {
  auto now = frc::Timer::GetFPGATimestamp();

  double throttle = ApplyDeadband(m_throttle(), kDeadband);
  double strafe = ApplyDeadband(m_strafe(), kDeadband);

  auto rpm = units::revolutions_per_minute_t{m_tunableRPM.Get()};
  auto hoodAngle = units::degree_t{m_tunableHoodAngle.Get()};

  m_flywheel->SetRPM(rpm);
  m_hood->SetPosition(hoodAngle);

  auto alliance = frc::DriverStation::GetAlliance();

  if(alliance.has_value() && alliance == frc::DriverStation::kBlue){
    m_targetHeading = 180_deg;
  }else{
    m_targetHeading = 0_deg;
  }

 double rotOutput =
      m_headingController.Calculate(m_drive->GetOdometryThread()
                                        ->GetEstimatedPose()
                                        .Rotation()
                                        .Radians()
                                        .value(),
                                   m_targetHeading.value());
  rotOutput =
      std::clamp(rotOutput, -Constants::SwerveDrive::kMaxAngularSpeed.value(),
                 Constants::SwerveDrive::kMaxAngularSpeed.value());


  auto maxSpeeds = m_drive->GetMaxSpeeds();
  units::meters_per_second_t maxLinearSpeed = maxSpeeds.first;
  auto xVel = throttle * maxLinearSpeed;
  auto yVel = strafe * maxLinearSpeed;
  m_drive->DriveFieldRelative(
      frc::ChassisSpeeds{xVel, yVel, units::radians_per_second_t{rotOutput}});

  bool flywheelReady = m_flywheel->AtSetpoint();
  bool hoodReady = m_hood->IsAtPosition(hoodAngle);
  bool ready = flywheelReady && hoodReady;

  auto &log = Logger::Instance();
  log.Log("PassCommand/FlywheelRPM", m_tunableRPM.Get());
  log.Log("PassCommand/HoodAngleDeg", m_tunableHoodAngle.Get());
  log.Log("PassCommand/Ready", ready);
  log.Log("PassCommand/ShootSequenceActive", m_shootSequenceActive);

  if (!m_shootSequenceActive && ready) {
    m_shootSequenceActive = true;
    m_slowRetractStarted = false;
    m_shootSequenceStartTime = now;
    m_deployer->FullyExtend();
  }

  if (m_shootSequenceActive && !m_slowRetractStarted &&
      now - m_shootSequenceStartTime >= kDeployerExtendDelay) {
    m_slowRetractStarted = true;
    m_deployer->SlowRetract(kSlowRetractDuration);
  }

  if (m_shootSequenceActive) {
    m_floor->SetVoltage(kFloorVoltage);
    m_feeder->SetVelocity(kFeederRPS);
    m_topRoller->SetVoltage(kTopVoltage);
    m_bottomRoller->SetVoltage(kBottomVoltage);
  } else {
    m_floor->Stop();
    m_feeder->Stop();
  }
}

void PassCommand::End(bool interrupted) {
  m_flywheel->SetRPM(0_rpm);
  m_hood->SetPosition(0_rad);
  m_topRoller->Stop();
  m_bottomRoller->Stop();
  m_feeder->Stop();
  m_floor->Stop();
  m_deployer->RetractMid();
  m_feeder->ClearIndexed();
  m_shootSequenceActive = false;
  m_slowRetractStarted = false;
}

bool PassCommand::IsFinished() { return false; }

double PassCommand::ApplyDeadband(double value, double deadband) {
  if (std::abs(value) < deadband)
    return 0.0;
  return (value - std::copysign(deadband, value)) / (1.0 - deadband);
}
