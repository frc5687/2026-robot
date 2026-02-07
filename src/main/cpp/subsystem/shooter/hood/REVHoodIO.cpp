#include "subsystem/shooter/hood/REVHoodIO.h"
#include "ctre/phoenix6/CANcoder.hpp"
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
    rev::servohub::ServoHub servoHub,
    const CANDevice &encoder
)
    : m_servoHub(servoHub.GetDeviceId()),
      m_servoChannel(
          servoHub.GetServoChannel(
              rev::servohub::ServoChannel::ChannelId::kChannelId0
          )
      ),
      m_encoder(encoder.id, encoder.bus),
      m_encoderAngle(m_encoder.GetAbsolutePosition())
{
    m_servoChannel.SetEnabled(true);
    m_servoChannel.SetPowered(true);
    m_encoderConfigs.MagnetSensor.MagnetOffset = Constants::Hood::kEncoderOffset;
    m_encoder.GetConfigurator().Apply(m_encoderConfigs);
}

void REVHoodIO::UpdateInputs(HoodIOInputs &inputs){
    m_encoderAngle.Refresh();

    inputs.hoodRotation = m_encoderAngle.GetValue();
    m_servoAngle = m_encoderAngle.GetValue();
    m_microseconds = m_servoChannel.GetPulseWidth();
}

void REVHoodIO::SetHoodPosition(units::turn_t hoodRotation){
    
    if(!frc::IsNear(hoodRotation, m_servoAngle, 0.1_tr)){
        (std::signbit(m_servoAngle.value() - hoodRotation.value())) ? 
        m_servoChannel.SetPulseWidth(std::clamp(m_microseconds + 10, 500, 2500)) 
        : m_servoChannel.SetPulseWidth(std::clamp(m_microseconds - 10, 500, 2500));
    }else if (!frc::IsNear(hoodRotation, m_servoAngle, 0.01_tr)){
        (std::signbit(m_servoAngle.value() - hoodRotation.value())) ? 
        m_servoChannel.SetPulseWidth(std::clamp(m_microseconds + 3, 500, 2500))
         : m_servoChannel.SetPulseWidth(std::clamp(m_microseconds - 3, 500, 2500));
    }
}

