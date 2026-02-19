// Team 5687 2026

#include "subsystem/indexer/IndexerSubsystem.h"

#include "Constants.h"
#include "RobotState.h"

using namespace Constants::Indexer;

IndexerSubsystem::IndexerSubsystem(std::unique_ptr<IndexerIO> io)
    : LoggedSubsystem("Indexer"), m_io(std::move(io)) {}

void IndexerSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void IndexerSubsystem::SetVelocity(units::turns_per_second_t rps) {
  m_io->SetMotorVelocity(rps);
}

void IndexerSubsystem::Stop() { m_io->Stop(); }

void IndexerSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);

  m_state.Update(m_inputs, Detection::kCurrentDeltaThreshold,
                 Detection::kVelocityDipThreshold,
                 Detection::kSeatedCurrentCeiling, Detection::kDebounceTime);

  RobotState::Instance().SetBallIndexingEvent(m_state.ballDetected);
}

void IndexerSubsystem::LogTelemetry() {
  Log("Velocity", m_inputs.motorVelocity.value());
  Log("Position", m_inputs.motorPosition.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("StatorCurrent", m_inputs.statorCurrent.value());

  Log("Center/Velocity", m_inputs.centerVelocity.value());
  Log("Center/StatorCurrent", m_inputs.centerStatorCurrent.value());

  Log("Detection/BallDetected", m_state.ballDetected);
  Log("Detection/BallSeated", m_state.ballSeated);
  Log("Detection/BallFed", m_state.ballFed);
  Log("Detection/CurrentDelta", m_state.currentDelta.value());
  Log("Detection/VelocityDelta", m_state.velocityDelta.value());
}
