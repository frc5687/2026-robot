// Team 5687 2026

#include "commands/intake/FlywheelHardstopCommand.h"

FlywheelHardstopCommand::FlywheelHardstopCommand(FlywheelSubsystem *flywheel)
    : m_flywheel(flywheel) {
  AddRequirements(flywheel);
  SetName("FlywheelHardstopCommand");
}

void FlywheelHardstopCommand::Initialize() {}

void FlywheelHardstopCommand::Execute() {
  if (m_flywheel->GetFilteredRPM() < kIdleThreshold) {
    m_flywheel->SetVoltage(kHardstopVoltage);
  }
}

void FlywheelHardstopCommand::End(bool interrupted) {}

bool FlywheelHardstopCommand::IsFinished() { return false; }
