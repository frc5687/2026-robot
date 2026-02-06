#pragma once


#include <frc/simulation/DCMotorSim.h>
#include <frc/controller/PIDController.h>

#include "subsystem/Indexer/IndexerIO.h"
#include "units/angular_velocity.h"

class SimIndexerIO : public IndexerIO {
 public:
  SimIndexerIO();
  ~SimIndexerIO() = default;

  void UpdateInputs(IndexerIOInputs& inputs) override;
  void SetVoltage(units::volt_t desiredVoltage, units::angular_velocity::turns_per_second_t rpm) override;

  // note to self: talk to denis about this
 private:
   frc::sim::DCMotorSim m_indexerSim;
  //  frc::PIDController m_controller;
};