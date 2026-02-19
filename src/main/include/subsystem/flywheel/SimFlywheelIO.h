// Team 5687 2026
#pragma once

#include <frc/controller/PIDController.h>
#include <frc/simulation/FlywheelSim.h>
#include "FlywheelIO.h"
#include "units/angular_velocity.h"

class SimFlywheelIO : public FlywheelIO {
 public:
  SimFlywheelIO();
  ~SimFlywheelIO() = default;

  void UpdateInputs(FlywheelIOInputs& inputs) override;
  void SetMotorRPM(units::revolutions_per_minute_t motorRPM) override;

 private:
  frc::sim::FlywheelSim m_flywheelSim;
  frc::PIDController m_controller;
  units::revolutions_per_minute_t m_desiredRPM{0_rpm};
};
