#pragma once

#include "HoodIO.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "rev/ServoChannel.h"
#include "units/angle.h"
#include "units/time.h"
#include "utils/CANDevice.h"
#include <rev/ServoHub.h>

class REVHoodIO : public HoodIO { 
public:
    REVHoodIO(
        const int& servoHubId,
        const CANDevice& leftEncoder,
        const CANDevice& rightEncoder
    );

    void UpdateInputs(HoodIOInputs& inputs);
    void SetHoodPosition(units::angle::turn_t leftHoodPosition, units::angle::turn_t rightHoodPosition);

private:
    rev::servohub::ServoHub m_servoHub;
    rev::servohub::ServoChannel m_leftServoChannel;
    rev::servohub::ServoChannel m_rightServoChannel;

    ctre::phoenix6::hardware::CANcoder m_leftEncoder;
    ctre::phoenix6::StatusSignal<units::turn_t> m_leftEncoderAngle;
    ctre::phoenix6::configs::CANcoderConfiguration m_leftEncoderConfigs;
    units::turn_t m_leftServoAngle{0_tr};
    int m_leftMicroseconds{1500};
    int m_leftStepSize{10};

    ctre::phoenix6::hardware::CANcoder m_rightEncoder;
    ctre::phoenix6::StatusSignal<units::turn_t> m_rightEncoderAngle;
    ctre::phoenix6::configs::CANcoderConfiguration m_rightEncoderConfigs;
    units::turn_t m_rightServoAngle{0_tr};
    int m_rightMicroseconds{1500};
    int m_rightStepSize{10};

    std::array<ctre::phoenix6::BaseStatusSignal*, 2> m_batchSignals;
};
