#include "subsystem/hood/HoodSubsystem.h"
#include "Constants.h"
#include "subsystem/LoggedSubsystem.h"
#include "RobotState.h"
#include <numbers>

HoodSubsystem::HoodSubsystem(std::unique_ptr<HoodIO> io) :
    LoggedSubsystem("Hood"), m_io(std::move(io)) {}

void HoodSubsystem::SetHoodPosition(units::turn_t desiredAngle) {
    m_desiredAngle = desiredAngle;
    m_io->SetHoodPosition(m_desiredAngle);
}

HoodState HoodSubsystem::GetHoodState() const {
    HoodState state;
    state.timestamp = m_inputs.timestamp;
    auto leftAngle = (m_inputs.leftHoodRotation / Constants::Hood::kGearRatio) *
                   (2.0 * std::numbers::pi * 1_rad) / 1_tr;
    auto rightAngle = (m_inputs.rightHoodRotation / Constants::Hood::kGearRatio) *
                   (2.0 * std::numbers::pi * 1_rad) / 1_tr;
    state.angle = (leftAngle + rightAngle) / 2.0;
    //state.angle = (m_inputs.leftHoodRotation
    return state;
}

void HoodSubsystem::UpdateInputs() {
    m_io->UpdateInputs(m_inputs);
    RobotState::Instance().AddHoodObservation(GetHoodState());
}

void HoodSubsystem::LogTelemetry() {
    Log("Left Servo Rotations", m_inputs.leftHoodRotation.value());
    Log("Left Servo usec", m_inputs.leftMicroseconds);

    Log("Right Servo Rotations", m_inputs.rightHoodRotation.value());
    Log("Right Servo usec", m_inputs.rightMicroseconds);
}

