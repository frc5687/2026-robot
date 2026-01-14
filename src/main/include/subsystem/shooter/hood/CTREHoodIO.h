#pragma once

#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include <utils/CANDevice.h>
#include "units/angle.h"
#include <ctre/phoenix6/TalonFX.hpp>


class CTREHoodIO : public HoodIO { 

    public:
        CTREHoodIO(const CANDevice& hoodmotor);
        SetHoodAngle(units::radians angle) override;

    private:
    
    ctre::phoenix6::hardware::TalonFX m_hoodMotor;
    
};