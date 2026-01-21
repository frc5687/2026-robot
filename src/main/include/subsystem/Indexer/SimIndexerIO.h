#pragma once


#include <frc/simulation/DCMotorSim.h>
#include <frc/controller/PIDController.h>

#include "subsystem/Indexer/IndexerIO.h"

class SimIndexerIO : public IndexerIO {
 public:
  SimIndexerIO();
  ~SimIndexerIO() = default;

  void UpdateInputs(IndexerIOInputs& inputs) override;
  void SetMotorSpeed(units::meters_per_second_t desiredSpeed) override;

  // note to self: talk to denis about this
 private:
   frc::sim::DCMotorSim m_indexerSim;
   frc::PIDController m_controller;
};