// Team 5687 2026

#include "commands/shooter/ManualShootCommand.h"

#include <frc/Timer.h>
#include <units/angle.h>

#include "utils/Logger.h"

ManualShootCommand::ManualShootCommand(
    FlywheelSubsystem *flywheel, HoodSubsystem *hood,
    IntakeTopRollerSubsystem *topRoller,
    IntakeBottomRollerSubsystem *bottomRoller, FeederSubsystem *feeder,
    FloorSubsystem *floor, IntakeDeployerSubsystem *deployer)
    : m_flywheel(flywheel), m_hood(hood), m_topRoller(topRoller),
      m_bottomRoller(bottomRoller), m_feeder(feeder), m_floor(floor),
      m_deployer(deployer) {
  AddRequirements({flywheel, hood, feeder, floor, deployer});
  SetName("ManualShootCommand");
}

void ManualShootCommand::Initialize() {
  m_shootSequenceActive = false;
  m_slowRetractStarted = false;
}

void ManualShootCommand::Execute() {
  auto now = frc::Timer::GetFPGATimestamp();

  auto rpm = units::revolutions_per_minute_t{m_tunableRPM.Get()};
  auto hoodAngle = units::degree_t{m_tunableHoodAngle.Get()};

  m_flywheel->SetRPM(rpm);
  m_hood->SetPosition(hoodAngle);

  bool flywheelReady = m_flywheel->AtSetpoint();
  bool hoodReady = m_hood->IsAtPosition(hoodAngle);
  bool ready = flywheelReady && hoodReady;

  auto &log = Logger::Instance();
  log.Log("ManualShoot/FlywheelRPM", m_tunableRPM.Get());
  log.Log("ManualShoot/HoodAngleDeg", m_tunableHoodAngle.Get());
  log.Log("ManualShoot/Ready", ready);
  log.Log("ManualShoot/ShootSequenceActive", m_shootSequenceActive);

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

void ManualShootCommand::End(bool interrupted) {
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

bool ManualShootCommand::IsFinished() { return false; }
