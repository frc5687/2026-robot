#pragma once

#include <frc/simulation/DCMotorSim.h>

#include "IndexerIO.h"
#include "frc/controller/ProfiledPIDController.h"

class SimIndexerIO : public IndexerIO {
 public:
  SimIndexerIO();
  ~SimIndexerIO() = default;

  void UpdateInputs(IndexerIOInputs& inputs) override;
  void SetMotorSpeed(units::meter_t desiredHeight) override;

  // note to self: talk to denis about this
 private:
   frc::sim::DCMotorSim m_indexerSim;
   frc::PIDController m_controller;
};