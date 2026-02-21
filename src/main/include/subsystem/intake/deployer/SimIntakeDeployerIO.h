// Team 5687 2026

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Constants.h"
#include "IntakeDeployerIO.h"

class SimIntakeDeployerIO : public IntakeDeployerIO {
public:
  SimIntakeDeployerIO();
  ~SimIntakeDeployerIO() override = default;

  void UpdateInputs(IntakeDeployerIOInputs &inputs) override;
  void SetPosition(units::turn_t position) override;
  void SetVoltage(units::volt_t voltage) override;
  void ZeroPosition() override;
  void Stop() override;

private:
  units::volt_t CalculateClosedLoop();

  frc::sim::DCMotorSim m_motorSim;
  frc::PIDController m_pid;

  units::turn_t m_position{0_tr};

  enum class Mode { kPosition, kVoltage, kStopped };
  Mode m_mode{Mode::kStopped};
  units::volt_t m_voltageCommand{0_V};
  units::turn_t m_positionSetpoint{0_tr};
};
