#pragma once

#include <frc/simulation/DCMotorSim.h>

#include "IntakeRollerIO.h"
#include "frc/controller/ProfiledPIDController.h"

class SimIntakeRollerIO : public IntakeRollerIO {
 public:
  SimIntakeRollerIO();
  ~SimIntakeRollerIO() = default;

  virtual void UpdateInputs(IntakeRollerIOInputs& inputs) = 0;
  virtual void SetIntakeRPM(units::revolutions_per_minute_t desiredRPM) = 0;

 private:
  frc::sim::DCMotorSim m_IntakeRollerSim;
  frc::ProfiledPIDController<units::meter> m_pidController;
};