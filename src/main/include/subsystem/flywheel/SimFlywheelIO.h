#pragma once

#include "FlywheelIO.h"
#include "units/angular_velocity.h"
#include <frc/simulation/FlywheelSim.h>
#include <frc/controller/PIDController.h>

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
