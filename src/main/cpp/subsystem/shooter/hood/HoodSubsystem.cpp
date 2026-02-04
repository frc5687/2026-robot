#include "subsystem/shooter/hood/HoodIO.h"
#include "subsystem/shooter/hood/HoodSubsystem.h"

HoodSubsystem::HoodSubsystem(std::unique_ptr<HoodIO>io):
LoggedSubsystem("Hood"), m_io(std::move(io)) {}

void HoodSubsystem::UpdateInputs() {m_io->UpdateInputs(m_inputs);}

void HoodSubsystem::SetHoodPosition(units::turn_t hoodRotation){
    m_io ->SetHoodPosition(hoodRotation);
}

void HoodSubsystem::LogTelemetry(){
    Log("Intake Position Meters", m_inputs.hoodRotation.value());
}