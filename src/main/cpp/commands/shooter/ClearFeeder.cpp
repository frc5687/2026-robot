// Team 5687 2026

#include "commands/shooter/ClearFeeder.h"

ClearFeeder::ClearFeeder(FeederSubsystem *feeder, FloorSubsystem *floor)
    : m_feeder(feeder), m_floor(floor) {
  AddRequirements({m_feeder, m_floor});
  SetName("ClearFeeder");
}

void ClearFeeder::Initialize() {}

void ClearFeeder::Execute() {
  m_feeder->SetVoltage(kFeederVoltage);
  m_floor->SetVoltage(kFloorVoltage);
}

void ClearFeeder::End(bool interrupted) {
  m_feeder->Stop();
  m_floor->Stop();
}

bool ClearFeeder::IsFinished() { return false; }
