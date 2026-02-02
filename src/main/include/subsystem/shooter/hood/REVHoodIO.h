#pragma once

#include "HoodIO.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "rev/ServoChannel.h"
#include "units/angle.h"
#include "units/time.h"
#include "utils/CANDevice.h"
#include <rev/ServoHub.h>

class REVHoodIO : public HoodIO { 
public:
    REVHoodIO(
        rev::servohub::ServoHub servoHub,
        const CANDevice& encoder
    );

    void UpdateInputs(HoodIOInputs& inputs);
    void SetHoodPosition(units::angle::turn_t hoodPosition);

private:
    rev::servohub::ServoHub m_servoHub;
    rev::servohub::ServoChannel m_servoChannel;

    ctre::phoenix6::hardware::CANcoder m_encoder;
    ctre::phoenix6::StatusSignal<units::turn_t> m_encoderAngle;

    units::turn_t m_servoAngle{0_tr};
    int m_microseconds{1500};
    int m_stepSize{10};
};
