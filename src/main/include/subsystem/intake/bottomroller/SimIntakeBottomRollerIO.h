// Team 5687 2026

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

#include "IntakeBottomRollerIO.h"
#include "subsystem/intake/IntakeConstants.h"

class SimIntakeBottomRollerIO : public IntakeBottomRollerIO {
public:
  SimIntakeBottomRollerIO();
  ~SimIntakeBottomRollerIO() override = default;

  void UpdateInputs(IntakeBottomRollerIOInputs &inputs) override;
  void SetVoltage(units::volt_t voltage) override;
  void SetCurrent(units::ampere_t current) override;
  void SetVelocity(units::turns_per_second_t rps) override;
  void Stop() override;

private:
  units::volt_t CalculateClosedLoop();

  frc::sim::DCMotorSim m_motorSim;
  frc::PIDController m_pid;

  enum class Mode { kVoltage, kVelocity, kStopped };
  Mode m_mode{Mode::kStopped};
  units::volt_t m_voltageCommand{0_V};
  units::turns_per_second_t m_velocitySetpoint{0_tps};
};
