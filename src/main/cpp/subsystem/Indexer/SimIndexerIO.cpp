#include "subsystem/Indexer/SimIndexerIO.h"

#include "frc/Timer.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "subsystem/Indexer/IndexerConstants.h"
#include "frc/system/plant/LinearSystemId.h"

SimIndexerIO::SimIndexerIO() :
    m_indexerSim(frc::LinearSystemId::DCMotorSystem(Constants::Indexer::kMotor, Constants::Indexer::kInertia, Constants::Indexer::kGearRatio), Constants::Indexer::kMotor, {0.001, 0.001}),
    m_controller(Constants::Indexer::kP, Constants::Indexer::kI, Constants::Indexer::kD) {}

// SimTurretIO::SimTurretIO() :
//     m_turretSim(frc::LinearSystemId::DCMotorSystem(kMotor, kInertia, kGearRatio), kMotor, {0.001, 0.001}),
//     m_controller(kP, kI, kD){}

void SimIndexerIO::UpdateInputs(IndexerIOInputs& inputs) {
  m_indexerSim.Update(20_ms);
  inputs.MotorPosition = m_indexerSim.GetAngularPosition();
  inputs.MotorVelocity = m_indexerSim.GetAngularVelocity();
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimIndexerIO::SetMotorSpeed(units::meters_per_second_t desiredSpeed) {
  auto position = m_indexerSim.GetAngularPosition();
  // auto pidOutput = m_controller.Calculate(position, desiredSpeed);
  // m_indexerSim.SetMotorVoltage(units::volt_t{pidOutput});

  // wip code
  // m_indexerSim.Set(pid.Calculate(encoder.GetDistance(), setpoint));
}
