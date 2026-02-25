// Team 5687 2026

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>

#include "Constants.h"
#include "HoodIO.h"

class SimHoodIO : public HoodIO {
public:
  SimHoodIO();
  ~SimHoodIO() override = default;

  void UpdateInputs(HoodIOInputs &inputs) override;
  void SetPosition(units::turn_t position) override;
  void SetVoltage(units::volt_t voltage) override;
  void ZeroPosition() override;
  void Stop() override;

private:
  units::volt_t CalculateClosedLoop();

  frc::sim::SingleJointedArmSim m_armSim;
  frc::PIDController m_pid;

  units::turn_t m_motorPosition{0_tr};

  enum class Mode { kPosition, kVoltage, kStopped };
  Mode m_mode{Mode::kStopped};
  units::volt_t m_voltageCommand{0_V};
  units::turn_t m_positionSetpoint{0_tr};
};
