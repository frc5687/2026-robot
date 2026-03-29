// Team 5687 2026

#include "commands/intake/IntakeCommand.h"

#include "Constants.h"
#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/drive/SwerveDriveConstants.h"
#include "subsystem/intake/IntakeConstants.h"

IntakeCommand::IntakeCommand(DriveSubsystem *drive,
                             IntakeDeployerSubsystem *deployer,
                             IntakeTopRollerSubsystem *topRoller,
                             IntakeBottomRollerSubsystem *bottomRoller,
                             FloorSubsystem *floor)
    : m_drive(drive), m_deployer(deployer), m_topRoller(topRoller),
      m_bottomRoller(bottomRoller), m_floor(floor) {
  AddRequirements({deployer, topRoller, bottomRoller, floor});
  SetName("IntakeCommand");
}

void IntakeCommand::Initialize() {
  // m_drive->SetMaxSpeeds(3.0_mps);
  m_deployer->Deploy();
}

void IntakeCommand::Execute() {
  m_deployer->Deploy();
  m_topRoller->SetVoltage(kTopRollerVoltage);
  m_bottomRoller->SetVoltage(kBottomRollerVoltage);
  m_floor->SetVoltage(kFloorVoltage);
}

void IntakeCommand::End(bool interrupted) {
  // m_deployer->RetractMid();
  // m_drive->SetMaxSpeeds(Constants::SwerveDrive::kMaxLinearSpeed);
  m_topRoller->Stop();
  m_bottomRoller->Stop();
  m_floor->Stop();
}

bool IntakeCommand::IsFinished() { return false; }
