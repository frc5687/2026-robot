#pragma once

#include <frc/simulation/DCMotorSim.h>

#include "LinearIntakeIO.h"
#include "frc/controller/PIDController.h"
#include "frc/controller/ProfiledPIDController.h"
#include "frc/simulation/LinearSystemSim.h"

class SimLinearIntakeIO : public LinearIntakeIO {
 public:
  SimLinearIntakeIO();
  ~SimLinearIntakeIO() = default;

  virtual void UpdateInputs(LinearIntakeIOInputs& inputs) override;
  virtual void SetPosition(units::meter_t desiredMeters) override;

 private:
  frc::sim::DCMotorSim m_linearIntakeSim;
  frc::PIDController m_pidController;
};