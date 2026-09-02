// Team 5687 2026

#include "commands/drive/SlowModeCommand.h"

#include "Constants.h"
#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/drive/SwerveDriveConstants.h"
#include "subsystem/intake/IntakeConstants.h"

SlowModeCommand::SlowModeCommand(DriveSubsystem *drive) : m_drive(drive) {
  SetName("SlowModeCommand");
}

void SlowModeCommand::Initialize() { m_drive->SetMaxSpeeds(2.0_mps); }

void SlowModeCommand::Execute() {}

void SlowModeCommand::End(bool interrupted) {
  // m_deployer->RetractMid();
  m_drive->SetMaxSpeeds(Constants::SwerveDrive::kMaxLinearSpeed);
}

bool SlowModeCommand::IsFinished() { return false; }
