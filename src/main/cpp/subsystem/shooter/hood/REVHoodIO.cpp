#include "subsystem/shooter/hood/REVHoodIO.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
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
    const CANDevice &hood,
    const CANDevice &encoder
)
    : m_hood(hood.id, hood.bus),
      m_encoder(encoder.id, encoder.bus),
      m_encoderAngle(m_encoder.GetPosition()),
      m_request(ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0))
{
    m_encoderConfigs.MagnetSensor.MagnetOffset = Constants::Hood::kEncoderOffset;
    m_encoderConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
    m_encoder.GetConfigurator().Apply(m_encoderConfigs);
    
    m_hoodConfigs.Feedback.FeedbackRemoteSensorID = encoder.id;
    m_hoodConfigs.Slot0.kP = 0.0;
    m_hoodConfigs.MotorOutput.Inverted = false;
    
}

void REVHoodIO::UpdateInputs(HoodIOInputs &inputs){
    m_encoderAngle.Refresh();

    inputs.hoodRotation = m_encoderAngle.GetValue();
    
}

void REVHoodIO::SetHoodPosition(units::turn_t hoodRotation){
    
   m_hood.SetPosition(hoodRotation * Constants::Hood::kGearRatio);
}

