// Team 5687 2026

#include "commands/shooter/AutoShootCommand.h"

#include <units/angular_velocity.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>

#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include "RobotState.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"


AutoShootCommand::AutoShootCommand(FlywheelSubsystem *flywheel,
                                   HoodSubsystem *hood, FeederSubsystem *feeder,
                                   KickerSubsystem *kicker,
                                   IntakeTopRollerSubsystem *topRoller,
                                   IntakeBottomRollerSubsystem *bottomRoller,
                                   IntakeDeployerSubsystem *deployer)
    : m_flywheel(flywheel), m_hood(hood), m_feeder(feeder), m_kicker(kicker),
      m_topRoller(topRoller), m_bottomRoller(bottomRoller),
      m_deployer(deployer) {
  AddRequirements({flywheel, hood, feeder, kicker, deployer});
  SetName("AutoShootCommand");
}

void AutoShootCommand::Initialize() {
  m_deployer->RetractMid();
  m_deployerExtended = false;
  m_pulseStartTime = frc::Timer::GetFPGATimestamp();
  pathplanner::PPHolonomicDriveController::overrideRotationFeedback([this]() {
    return m_rotationFeedback;
  });
}

void AutoShootCommand::Execute() {


  auto now = frc::Timer::GetFPGATimestamp();
  auto alliance = frc::DriverStation::GetAlliance();
  bool isRed = alliance == frc::DriverStation::Alliance::kRed;

  auto solution = m_shotCalculator.Calculate(now, isRed);

  m_flywheel->SetRPM(units::revolutions_per_minute_t{solution.flywheelSpeed});
  m_hood->SetPosition(units::radian_t{solution.hoodAngle});

  auto elapsed = now - m_pulseStartTime;

  double rotOutput =
      m_headingController.Calculate(RobotState::Instance().GetDriveState(now).estimatedPose
                                        .Rotation()
                                        .Radians()
                                        .value(),
                                    solution.driveAngle.Radians().value());
  rotOutput =
      std::clamp(rotOutput, -Constants::SwerveDrive::kMaxAngularSpeed.value(),
                 Constants::SwerveDrive::kMaxAngularSpeed.value());

  m_rotationFeedback = units::radians_per_second_t{rotOutput};



  if (m_deployerExtended) {
    if (elapsed >= kPulseExtendDuration) {
      m_deployer->RetractMid();
      m_deployerExtended = false;
      m_pulseStartTime = now;
    }
  } else {
    if (elapsed >= kPulseRetractDuration) {
      m_deployer->Deploy();
      m_deployerExtended = true;
      m_pulseStartTime = now;
    }
  }

  if (solution.ready) {
    m_feeder->SetVoltage(kFeedVoltage);
    m_topRoller->SetVoltage(kTopVoltage);
    m_bottomRoller->SetVoltage(kBottomVoltage);
    m_kicker->SetVelocity(kKickerRPS);
  } else {
    m_feeder->Stop();
    m_kicker->Stop();
  }
}

void AutoShootCommand::End(bool interrupted) {
  m_flywheel->SetRPM(0_rpm);
  m_hood->SetPosition(0_rad);
  m_topRoller->Stop();
  m_bottomRoller->Stop();
  m_feeder->Stop();
  m_kicker->Stop();
  m_deployer->RetractMid();

  pathplanner::PPHolonomicDriveController::clearRotationFeedbackOverride();
}

bool AutoShootCommand::IsFinished() { return false; }
