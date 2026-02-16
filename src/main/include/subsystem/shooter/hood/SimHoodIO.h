#pragma once

#include "HoodIO.h"
#include "frc/controller/PIDController.h"
#include "frc/simulation/SingleJointedArmSim.h"
#include "units/angle.h"


class SimHoodIO : public HoodIO {

    public:
        SimHoodIO();
        ~SimHoodIO() = default;

        virtual void UpdateInputs(HoodIOInputs& inputs) override;
        virtual void SetHoodPosition(units::angle::turn_t hoodPosition) override;

    private:
        frc::sim::SingleJointedArmSim m_simHood;
        frc::PIDController m_pidController;
};