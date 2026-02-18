// Team 5687 2026

#pragma once

#include <frc/simulation/FlywheelSim.h>

#include "FlywheelIO.h"
#include "frc/controller/PIDController.h"
#include "units/angular_velocity.h"

class SimFlywheelIO : public FlywheelIO {
 public:
  SimFlywheelIO();
  ~SimFlywheelIO() = default;

  void UpdateInputs(FlywheelIOInputs& inputs) override;
  void SetFlywheelRPM(units::revolutions_per_minute_t desiredRPM) override;

 private:
  frc::sim::FlywheelSim m_flywheelSim;
  frc::PIDController m_controller;
};
