// Team 5687 2026

#pragma once

#include "HoodIO.h"
#include "frc/controller/PIDController.h"
#include "frc/simulation/SingleJointedArmSim.h"
#include "units/angle.h"

class SimHoodIO : public HoodIO {

public:
  SimHoodIO();
  ~SimHoodIO() = default;

  void UpdateInputs(HoodIOInputs &inputs) override;
  void SetHoodPosition(units::angle::turn_t hoodPosition) override;
  void SetHoodPosition(units::turn_t leftHoodPosition,
                       units::turn_t rightHoodPosition) override;

private:
  frc::sim::SingleJointedArmSim m_simHood;
  frc::PIDController m_pidController;
  units::turn_t m_desiredRotation{0_tr};
};
