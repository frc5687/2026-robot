// Team 5687 2026

#include "commands/shooter/SimpleShootCommand.h"

#include "units/angular_velocity.h"

SimpleShootCommand::SimpleShootCommand(
    FlywheelSubsystem *flywheel, KickerSubsystem *kicker,
    FloorRollerSubsystem *floorRoller, HoodSubsystem *hood,
    IntakeBottomRollerSubsystem *bottomRoller,
    units::revolutions_per_minute_t flywheelRPM,
    units::turns_per_second_t kickerRPS, units::degree_t hoodAngle)
    : m_flywheel(flywheel), m_kicker(kicker), m_floorRoller(floorRoller),
      m_hood(hood), m_bottomRoller(bottomRoller), m_flywheelRPM(flywheelRPM), m_kickerRPS(kickerRPS),
      m_hoodAngle(hoodAngle) {
  AddRequirements({flywheel, kicker, floorRoller});
  SetName("SimpleShootCommand");
}

void SimpleShootCommand::Initialize() {
  m_flywheel->SetRPM(m_flywheelRPM);
  m_kicker->SetVelocity(m_kickerRPS);
  m_hood->SetPosition(m_hoodAngle);
}

void SimpleShootCommand::Execute() {
  m_flywheel->SetRPM(m_flywheelRPM);
  m_kicker->SetVelocity(m_kickerRPS);
  if (m_flywheel->AtSetpoint()) {
    m_floorRoller->SetVoltage(kFloorRollerVoltage);
    m_bottomRoller->SetVoltage(kBottomVoltage);
  } else {
    m_floorRoller->Stop();
  }
}

void SimpleShootCommand::End(bool interrupted) {
  m_flywheel->SetRPM(0_rpm);
  m_kicker->Stop();
  m_floorRoller->Stop();
  m_floorRoller->Stop();
  m_hood->SetPosition(0_deg);
}

bool SimpleShootCommand::IsFinished() { return false; }
