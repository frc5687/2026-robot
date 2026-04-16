// Team 5687 2026

#include "commands/intake/IndexCommand.h"

IndexCommand::IndexCommand(FeederSubsystem *feeder, FloorSubsystem *floor)
    : m_feeder(feeder), m_floor(floor) {
  AddRequirements({feeder, floor});
  SetName("IndexCommand");
}

void IndexCommand::Initialize() {}

void IndexCommand::Execute() {
  if (m_feeder->ShouldIndex()) {
    m_feeder->SetVoltage(kFeederVoltage);
    m_floor->SetVoltage(kFloorVoltage);
  } else {
    m_feeder->Stop();
    m_floor->Stop();
  }
}

void IndexCommand::End(bool interrupted) {
  m_feeder->Stop();
  m_floor->Stop();
}

bool IndexCommand::IsFinished() { return false; }
