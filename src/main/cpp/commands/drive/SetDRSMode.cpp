// Team 5687 2026

#include "commands/drive/SetDRSMode.h"

SetDRSMode::SetDRSMode(DriveSubsystem *drive) : m_drive(drive) {
  SetName("SetDRSMode");
}

void SetDRSMode::Initialize() { m_drive->SetDRSMode(); }

void SetDRSMode::Execute() {}

void SetDRSMode::End(bool interrupted) { m_drive->SetTeleopCurrentLimits(); }

bool SetDRSMode::IsFinished() { return false; }
