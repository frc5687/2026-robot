// Team 5687 2026

#pragma once

#include <frc/simulation/FlywheelSim.h>

#include "IntakeIO.h"

class SimIntakeIO : public IntakeIO {
public:
  SimIntakeIO();
  void UpdateInputs(IntakeIOInputs &inputs) override;
  void SetVoltage(units::volt_t voltage) override;
  void Stop() override;

private:
  // frc::sim::FlywheelSim m_sim;
  units::volt_t m_voltage;
};
