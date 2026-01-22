#pragma once

#include <frc/simulation/DCMotorSim.h>

#include "IntakeRollerIO.h"
#include "frc/controller/ProfiledPIDController.h"

class SimIntakeRollerIO : public IntakeRollerIO {
 public:
  SimIntakeRollerIO();
  ~SimIntakeRollerIO() = default;

  void UpdateInputs(IntakeRollerIOInputs& inputs) = 0;
  void SetIntakeRPM(units::radians_per_second_t DesiredAngularVelocity);

 private:
  frc::sim::DCMotorSim m_IntakeRollerSim;
  frc::ProfiledPIDController<units::radians_per_second> m_pidController;

};