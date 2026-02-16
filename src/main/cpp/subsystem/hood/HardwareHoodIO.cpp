#include "subsystem/shooter/hood/REVHoodIO.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "rev/ServoChannel.h"
#include "rev/ServoHub.h"
#include "rev/config/ServoChannelConfig.h"
#include "rev/config/ServoHubConfig.h"
#include "subsystem/shooter/hood/HoodIO.h"
#include "frc/MathUtil.h"
#include "utils/CANDevice.h"
#include "utils/TunableDouble.h"
#include <algorithm>
#include <cmath>
#include "subsystem/shooter/hood/HoodConstants.h"


REVHoodIO::REVHoodIO(
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
      m_leftEncoderAngle(m_leftEncoder.GetPosition()),

      m_rightEncoder(rightEncoder.id, rightEncoder.bus),
      m_rightEncoderAngle(m_rightEncoder.GetPosition()),

    m_batchSignals{&m_leftEncoderAngle, &m_rightEncoderAngle}
{
    m_leftServoChannel.SetEnabled(true);
    m_leftServoChannel.SetPowered(true);
    m_leftEncoderConfigs.MagnetSensor.MagnetOffset = Constants::Hood::kLeftEncoderOffset;
    m_leftEncoderConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
    m_leftEncoder.GetConfigurator().Apply(m_leftEncoderConfigs);

    m_rightServoChannel.SetEnabled(true);
    m_rightServoChannel.SetPowered(true);
    m_rightEncoderConfigs.MagnetSensor.MagnetOffset = Constants::Hood::kRightEncoderOffset;
    m_rightEncoderConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
    m_rightEncoder.GetConfigurator().Apply(m_rightEncoderConfigs);
}

void REVHoodIO::UpdateInputs(HoodIOInputs &inputs){
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

void REVHoodIO::SetHoodPosition(units::turn_t leftHoodRotation, units::turn_t rightHoodRotation){
    
    if(!frc::IsNear(leftHoodRotation, m_leftServoAngle, 0.1_tr)){
        (std::signbit(m_leftServoAngle.value() - leftHoodRotation.value())) ? 
        m_leftServoChannel.SetPulseWidth(std::clamp(m_leftMicroseconds + 200, 500, 2500)) 
        : m_leftServoChannel.SetPulseWidth(std::clamp(m_leftMicroseconds - 200, 500, 2500));
    }else if (!frc::IsNear(leftHoodRotation, m_leftServoAngle, 0.01_tr)){
        (std::signbit(m_leftServoAngle.value() - leftHoodRotation.value())) ? 
        m_leftServoChannel.SetPulseWidth(std::clamp(m_leftMicroseconds + 25, 500, 2500))
         : m_leftServoChannel.SetPulseWidth(std::clamp(m_leftMicroseconds - 25, 500, 2500));
    }

    if(!frc::IsNear(rightHoodRotation, m_rightServoAngle, 0.1_tr)){
        (std::signbit(m_rightServoAngle.value() - rightHoodRotation.value())) ? 
        m_rightServoChannel.SetPulseWidth(std::clamp(m_rightMicroseconds + 200, 500, 2500)) 
        : m_rightServoChannel.SetPulseWidth(std::clamp(m_rightMicroseconds - 200, 500, 2500));
    }else if (!frc::IsNear(rightHoodRotation, m_rightServoAngle, 0.01_tr)){
        (std::signbit(m_rightServoAngle.value() - rightHoodRotation.value())) ? 
        m_rightServoChannel.SetPulseWidth(std::clamp(m_rightMicroseconds + 25, 500, 2500))
         : m_rightServoChannel.SetPulseWidth(std::clamp(m_rightMicroseconds - 25, 500, 2500));
    }
}

