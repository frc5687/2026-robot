#include "subsystem/shooter/hood/HoodIO.h"
#include "subsystem/shooter/hood/HoodSubsystem.h"

HoodSubsystem::HoodSubsystem(std::unique_ptr<HoodIO>io):
LoggedSubsystem("Hood"), m_io(std::move(io)) {}

void HoodSubsystem::UpdateInputs() {m_io->UpdateInputs(m_inputs);}

void HoodSubsystem::SetHoodPosition(units::turn_t leftHoodRotation, units::turn_t rightHoodRotation){
    m_io ->SetHoodPosition(hoodRotation, rightHoodRotation);
}

void HoodSubsystem::LogTelemetry(){
    Log("left hood Position Meters", m_inputs.leftHoodRotation.value());
    Log("right hood Position Meters", m_inputs.rightHoodRotation.value());
    Log("left microseconds", m_inputs.leftMicroseconds);
    Log("right microseconds", m_inputs.rightMicroseconds);

}