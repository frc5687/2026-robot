// Team 5687 2026

#pragma once

#include <frc/simulation/FlywheelSim.h>

#include "FlywheelIO.h"
#include "frc/controller/PIDController.h"

class SimFlywheelIO : public FlywheelIO {
 public:
  SimFlywheelIO();
  ~SimFlywheelIO() = default;

  void UpdateInputs(FlywheelIOInputs& inputs) override;

 private:
  frc::sim::FlywheelSim m_flywheelSim;
  frc::PIDController m_controller;
};
