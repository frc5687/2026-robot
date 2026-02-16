#include "subsystem/hood/HoodIO.h"
#include "subsystem/hood/HoodSubsystem.h"

HoodSubsystem::HoodSubsystem(std::unique_ptr<HoodIO>io):
LoggedSubsystem("Hood"), m_io(std::move(io)) {}

void HoodSubsystem::UpdateInputs() {m_io->UpdateInputs(m_inputs);}

void HoodSubsystem::SetHoodPosition(units::turn_t leftHoodRotation,units::turn_t rightHoodRotation){
    m_io ->SetHoodPosition(leftHoodRotation,rightHoodRotation);
}

void HoodSubsystem::LogTelemetry(){
    Log("leftHoodRotation in turn_t", m_inputs.leftHoodRotation.value());
    Log("rightHoodRotation in turn_t", m_inputs.rightHoodRotation.value());

    Log("left microseconds", m_inputs.leftMicroseconds);
    Log("right microseconds", m_inputs.rightMicroseconds);
}