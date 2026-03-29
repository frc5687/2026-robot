// Team 5687 2026

#include "commands/shooter/SimpleShootCommand.h"

#include <frc/Timer.h>

#include "subsystem/intake/IntakeConstants.h"
#include "units/angular_velocity.h"

SimpleShootCommand::SimpleShootCommand(
    FlywheelSubsystem *flywheel, FeederSubsystem *feeder, FloorSubsystem *floor,
    HoodSubsystem *hood, IntakeBottomRollerSubsystem *bottomRoller,
    IntakeDeployerSubsystem *deployer,
    units::revolutions_per_minute_t flywheelRPM,
    units::turns_per_second_t kickerRPS, units::degree_t hoodAngle)
    : m_flywheel(flywheel), m_feeder(feeder), m_floor(floor), m_hood(hood),
      m_bottomRoller(bottomRoller), m_deployer(deployer),
      m_flywheelRPM(flywheelRPM), m_kickerRPS(kickerRPS),
      m_hoodAngle(hoodAngle),
      m_tunableFlywheelRPM("SimpleShoot", "FlywheelRPM", flywheelRPM.value()),
      m_tunableKickerRPS("SimpleShoot", "KickerRPM", kickerRPS.value()),
      m_tunableHoodAngle("SimpleShoot", "HoodAngleDeg", hoodAngle.value()),
      m_tunableBottomVoltage("SimpleShoot", "BottomVoltage",
                             kBottomVoltage.value()),
      m_tunableFloorVoltage("SimpleShoot", "FloorVoltage",
                            kFloorVoltage.value()) {
  AddRequirements({flywheel, feeder, floor, deployer});
  SetName("SimpleShootCommand");
}

void SimpleShootCommand::Initialize() {
  m_flywheel->SetRPM(m_flywheelRPM);
  m_feeder->SetVelocity(m_kickerRPS);
  m_hood->SetPosition(m_hoodAngle);
  m_deployer->RetractMid();
  m_deployerExtended = false;
  m_pulseStartTime = frc::Timer::GetFPGATimestamp();
}

void SimpleShootCommand::Execute() {
  if (m_tunableFlywheelRPM.HasChanged()) {
    m_flywheelRPM = units::revolutions_per_minute_t{m_tunableFlywheelRPM.Get()};
  }

  if (m_tunableKickerRPS.HasChanged()) {
    m_kickerRPS = units::turns_per_second_t{m_tunableKickerRPS.Get()};
  }

  if (m_tunableHoodAngle.HasChanged()) {
    m_hoodAngle = units::degree_t{m_tunableHoodAngle.Get()};
    m_hood->SetPosition(m_hoodAngle);
  }

  m_flywheel->SetRPM(m_flywheelRPM);
  m_feeder->SetVelocity(m_kickerRPS);

  auto now = frc::Timer::GetFPGATimestamp();
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

  if (m_tunableFloorVoltage.HasChanged() ||
      m_tunableBottomVoltage.HasChanged()) {
    m_floorVolts = units::volt_t{m_tunableFloorVoltage.Get()};
    m_bottomVolts = units::volt_t{m_tunableBottomVoltage.Get()};
  }

  if (m_flywheel->AtSetpoint()) {
    m_floor->SetVoltage(m_floorVolts);
    m_bottomRoller->SetVoltage(m_bottomVolts);
  } else {
    m_floor->Stop();
  }
}

void SimpleShootCommand::End(bool interrupted) {
  m_flywheel->SetRPM(0_rpm);
  m_feeder->Stop();
  m_floor->Stop();
  m_hood->SetPosition(0_deg);
  m_bottomRoller->Stop();
  m_deployer->RetractMid();
}

bool SimpleShootCommand::IsFinished() { return false; }
