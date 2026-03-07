// Team 5687 2026

#include "commands/shooter/AutoSpinUpCommand.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>


AutoSpinUpCommand::AutoSpinUpCommand(FlywheelSubsystem *flywheel)
    : m_flywheel(flywheel){
  AddRequirements(flywheel);
  SetName("AutoSpinUpCommand");
}

void AutoSpinUpCommand::Initialize() {
 
}

void AutoSpinUpCommand::Execute() {
  m_flywheel->SetRPM(1100_rpm);
}

void AutoSpinUpCommand::End(bool interrupted) {
}

bool AutoSpinUpCommand::IsFinished() { return false; }
