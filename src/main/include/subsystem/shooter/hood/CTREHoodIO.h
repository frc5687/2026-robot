#pragma once

#include "HoodIO.h"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include <utils/CANDevice.h>
#include "units/angle.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Servo.h>

class CTREHoodIO : public HoodIO { 

    public:
        CTREHoodIO(const CANDevice& hoodmotor);
        void UpdateInputs(HoodIOInputs& inputs);
        void setHoodPosition(units::angle::turn_t hoodPosition) override;
        
    private:

    frc::Servo m_servo;
    ctre::phoenix6::hardware::TalonFX m_hoodMotor;
    
};