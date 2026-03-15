// Team 5687 2026

#include "commands/shooter/ClearFeeder.h"

ClearFeeder::ClearFeeder(FeederSubsystem *feeder) : m_feeder(feeder) {
  AddRequirements({m_feeder, m_feeder});
  SetName("ClearFeeder");
}

void ClearFeeder::Initialize() {}

void ClearFeeder::Execute() {
  m_feeder->SetVoltage(kFeederVoltage);
}

void ClearFeeder::End(bool interrupted) {
  m_feeder->Stop();
}

bool ClearFeeder::IsFinished() { return false; }
