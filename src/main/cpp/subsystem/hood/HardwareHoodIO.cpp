

#include "subsystem/hood/HardwareHoodIO.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "rev/ServoChannel.h"
#include "rev/ServoHub.h"
#include "rev/config/ServoChannelConfig.h"
#include "rev/config/ServoHubConfig.h"

#include "frc/MathUtil.h"
#include "utils/CANDevice.h"
#include "utils/TunableDouble.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>
#include "Constants.h"

HardwareHoodIO::HardwareHoodIO(
    const int& servoHubId,
    const CANDevice &leftEncoder,
    const CANDevice &rightEncoder
)
    : m_servoHub(servoHubId),
      m_leftServoChannel(
          m_servoHub.GetServoChannel(
              rev::servohub::ServoChannel::ChannelId::kChannelId0

          )
      ),
      m_rightServoChannel(m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId1)),
      
      m_leftEncoder(leftEncoder.id, leftEncoder.bus),
      m_leftEncoderAngle(m_leftEncoder.GetAbsolutePosition()),

      m_rightEncoder(rightEncoder.id, rightEncoder.bus),
      m_rightEncoderAngle(m_rightEncoder.GetAbsolutePosition()),

    m_bigStep("stepsize", "bigstep", Constants::Hood::bigstep),
    m_littleStep("stepsize", "littlestep", Constants::Hood::littestep),

    m_batchSignals{&m_leftEncoderAngle, &m_rightEncoderAngle}

{
    m_leftMicrosecondMap.InsertValues({{0.0068, 2200}, {0.0678, 2000},{0.1467, 1800},{0.2241, 1600},{0.2975, 1400}, {0.3728, 1200}, {0.4443, 1000}, {0.5175, 800}, {0.5573, 700}});
    m_rightMicrosecondMap.InsertValues({{0.009, 2125},{0.050, 2000},{0.131, 1800},{0.203, 1600},{0.2702, 1400}, {0.338, 1200}, {0.4138, 1000}, {0.4953, 800}, {0.5576, 650}});

    m_leftServoChannel.SetEnabled(true);
    m_leftServoChannel.SetPowered(true);

    m_rightServoChannel.SetEnabled(true);
    m_rightServoChannel.SetPowered(true);
    
    m_leftServoChannel.SetPulseWidth(0);
    m_rightServoChannel.SetPulseWidth(0);

    m_leftEncoderConfigs.MagnetSensor.MagnetOffset = Constants::Hood::kLeftEncoderOffset;
    m_leftEncoderConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
    m_leftEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0_tr;
    m_leftEncoder.GetConfigurator().Apply(m_leftEncoderConfigs);

    m_rightEncoderConfigs.MagnetSensor.MagnetOffset = Constants::Hood::kRightEncoderOffset;
    m_rightEncoderConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
    m_rightEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0_tr;
    m_rightEncoder.GetConfigurator().Apply(m_rightEncoderConfigs);
}

void HardwareHoodIO::UpdateInputs(HoodIOInputs &inputs){
    ctre::phoenix6::BaseStatusSignal::RefreshAll(m_batchSignals);

    inputs.leftHoodRotation = m_leftEncoderAngle.GetValue();
    m_leftServoAngle = m_leftEncoderAngle.GetValue();
    inputs.leftMicroseconds = m_leftServoChannel.GetPulseWidth();
    m_leftMicroseconds = m_leftServoChannel.GetPulseWidth();

    inputs.rightHoodRotation = m_rightEncoderAngle.GetValue();
    m_rightServoAngle = m_rightEncoderAngle.GetValue();
    inputs.rightMicroseconds = m_rightServoChannel.GetPulseWidth();
    m_rightMicroseconds = m_rightServoChannel.GetPulseWidth();
}

void HardwareHoodIO::SetMicroseconds(double microsecond){
    int m_microsecond = static_cast<int>(microsecond);
    m_leftServoChannel.SetPulseWidth(m_microsecond);
    m_rightServoChannel.SetPulseWidth(m_microsecond);
}

 void HardwareHoodIO::SetMicrosecondMap(std::vector<std::pair<double, int>>leftMap, std::vector<std::pair<double, int>>rightMap){
    m_leftMicrosecondMap = {};
    m_rightMicrosecondMap = {};

    m_leftMicrosecondMap.InsertValues(leftMap);
    m_rightMicrosecondMap.InsertValues(rightMap);
 }

void HardwareHoodIO::SetHoodPosition(units::turn_t hoodposition){

}

void HardwareHoodIO::SetHoodPosition(units::turn_t leftHood, units::turn_t rightHood){

    int leftTargetmicroseconds = m_leftMicrosecondMap.GetValue(std::clamp(leftHood.value(), 0.01, 0.55));
    int rightTargetmicroseconds = m_rightMicrosecondMap.GetValue(std::clamp(rightHood.value(), 0.01, 0.55));

    m_leftServoChannel.SetPulseWidth(leftTargetmicroseconds);
    m_rightServoChannel.SetPulseWidth(rightTargetmicroseconds);
    // auto leftHoodRotation = std::clamp(leftHood, Constants::Hood::kMinAngle, Constants::Hood::kMaxAngle);
    // auto rightHoodRotation = std::clamp(rightHood, Constants::Hood::kMinAngle, Constants::Hood::kMaxAngle);
    // int bigstep = static_cast<int>(m_bigStep.Get());
    // int littlestep = static_cast<int>(m_littleStep.Get());
    // if(!frc::IsNear(leftHoodRotation, m_leftServoAngle, 0.1_tr)){
    //     (std::signbit(m_leftServoAngle.value() - leftHoodRotation.value())) ? 
    //     m_leftServoChannel.SetPulseWidth(std::clamp(m_leftMicroseconds - bigstep, 500, 2500)) 
    //     : m_leftServoChannel.SetPulseWidth(std::clamp(m_leftMicroseconds + bigstep, 500, 2500));
    // }else if (!frc::IsNear(leftHoodRotation, m_leftServoAngle, 0.01_tr)){
    //     (std::signbit(m_leftServoAngle.value() - leftHoodRotation.value())) ? 
    //     m_leftServoChannel.SetPulseWidth(std::clamp(m_leftMicroseconds - littlestep, 500, 2500))
    //      : m_leftServoChannel.SetPulseWidth(std::clamp(m_leftMicroseconds + littlestep, 500, 2500));
    // }

    // if(!frc::IsNear(rightHoodRotation, m_rightServoAngle, 0.1_tr)){
    //     (std::signbit(m_rightServoAngle.value() - rightHoodRotation.value())) ? 
    //     m_rightServoChannel.SetPulseWidth(std::clamp(m_rightMicroseconds - bigstep, 500, 2500)) 
    //     : m_rightServoChannel.SetPulseWidth(std::clamp(m_rightMicroseconds + bigstep, 500, 2500));
    // }else if (!frc::IsNear(rightHoodRotation, m_rightServoAngle, 0.01_tr)){
    //     (std::signbit(m_rightServoAngle.value() - rightHoodRotation.value())) ? 
    //     m_rightServoChannel.SetPulseWidth(std::clamp(m_rightMicroseconds - littlestep, 500, 2500))
    //      : m_rightServoChannel.SetPulseWidth(std::clamp(m_rightMicroseconds + littlestep, 500, 2500));
    // }
}