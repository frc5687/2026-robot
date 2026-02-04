// #include "subsystem/Indexer/SimIndexerIO.h"

// #include "frc/Timer.h"
// #include "subsystem/Indexer/IndexerConstants.h"
// #include <frc/system/plant/LinearSystemId.h>

// SimIndexerIO::SimIndexerIO() :
//     m_indexerSim(frc::LinearSystemId::DCMotorSystem(Constants::Indexer::kMotor, Constants::Indexer::kInertia, Constants::Indexer::kGearRatio), Constants::Indexer::kMotor, {0.001, 0.001}) {}

// // SimTurretIO::SimTurretIO() :
// //     m_turretSim(frc::LinearSystemId::DCMotorSystem(kMotor, kInertia, kGearRatio), kMotor, {0.001, 0.001}),
// //     m_controller(kP, kI, kD){}

// void SimIndexerIO::UpdateInputs(IndexerIOInputs& inputs) {
//   m_indexerSim.Update(20_ms);
//   inputs.MotorPosition = m_indexerSim.GetAngularPosition();
//   inputs.MotorVelocity = m_indexerSim.GetAngularVelocity();
//   inputs.timestamp = frc::Timer::GetFPGATimestamp();
// }

// void SimIndexerIO::SetVoltage(units::volt_t desiredVoltage) {
//   m_indexerSim.SetInputVoltage(desiredVoltage);
// }
