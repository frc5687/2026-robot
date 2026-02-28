// Team 5687 2026

#include "commands/shooter/AutoShootCommand.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>

AutoShootCommand::AutoShootCommand(FlywheelSubsystem *flywheel,
                                   HoodSubsystem *hood, FeederSubsystem *feeder,
                                   KickerSubsystem *kicker,
                                   IntakeBottomRollerSubsystem *bottomRoller,
                                   IntakeDeployerSubsystem *deployer)
    : m_flywheel(flywheel), m_hood(hood), m_feeder(feeder), m_kicker(kicker),
      m_bottomRoller(bottomRoller), m_deployer(deployer) {
  AddRequirements({flywheel, hood, feeder, kicker, deployer});
  SetName("AutoShootCommand");
}

void AutoShootCommand::Initialize() {
  m_deployer->RetractMid();
  m_deployerExtended = false;
  m_pulseStartTime = frc::Timer::GetFPGATimestamp();
}

void AutoShootCommand::Execute() {
  auto now = frc::Timer::GetFPGATimestamp();
  bool isRed = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;

  auto solution = m_shotCalculator.Calculate(now, isRed);

  m_flywheel->SetRPM(units::revolutions_per_minute_t{solution.flywheelSpeed});
  m_hood->SetPosition(units::radian_t{solution.hoodAngle});

  auto elapsed = now - m_pulseStartTime;

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
  m_bottomRoller->Stop();
  m_feeder->Stop();
  m_kicker->Stop();
  m_deployer->RetractMid();
}

bool AutoShootCommand::IsFinished() { return false; }
