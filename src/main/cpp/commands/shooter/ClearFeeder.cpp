// Team 5687 2026

#include "commands/shooter/ClearFeeder.h"
#include "subsystem/kicker/KickerSubsystem.h"

ClearFeeder::ClearFeeder(
    KickerSubsystem *kicker, FeederSubsystem *feeder) :
      m_kicker(kicker), m_feeder(feeder) {
  AddRequirements({kicker, feeder});
  SetName("IntakeCommand");
}

void ClearFeeder::Initialize() {}

void ClearFeeder::Execute() {
  m_kicker->SetVoltage(kKickerVoltage);
  m_feeder->SetVoltage(kFeederVoltage);
}

void ClearFeeder::End(bool interrupted) {
  // m_deployer->RetractMid();
  m_feeder->Stop();
  m_kicker->Stop();
}

bool ClearFeeder::IsFinished() { return false; }
