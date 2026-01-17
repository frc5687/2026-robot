#include "subsystem/SimIndexerIO.h"

#include "frc/Timer.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "subsystem/elevator/IndexerConstants.h"

SimIndexerIO::SimIndexerIO()
    : m_indexerSim(Constants::Indexer::kMotor,
                    Constants::Indexer::kGearRatio, Constants::Indexer::kMass,
                    Constants::Indexer::kDrumRadius,
                    Constants::Indexer::kMinHeight,
                    Constants::Indexer::kMaxHeight, true, 0_m, {0.001, 0.001}),
      m_pidController(100, 0, 0,
                      frc::TrapezoidProfile<units::meter>::Constraints(
                          Constants::Indexer::kMaxVelocity,
                          Constants::Indexer::kMaxAccel)) {}

void SimIndexerIO::UpdateInputs(IndexerIOInputs& inputs) {
  m_indexerSim.Update(20_ms);
  inputs.indexerPosition = m_indexerSim.GetPosition();
  inputs.indexerVelocity = m_indexerSim.GetVelocity();
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimIndexerIO::SetIndexerSpeed(units::meter_per_second_t desiredSpeed) {
  auto position = m_indexerSim.GetPosition();
  auto pidOutput = m_pidController.Calculate(position, desiredHeight);

  m_indexerSim.SetInputVoltage(units::volt_t{pidOutput});
}