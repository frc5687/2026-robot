// Team 5687 2026

#pragma once

#include <rev/ServoHub.h>

#include "HoodIO.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "rev/ServoChannel.h"
#include "units/angle.h"
#include "utils/CANDevice.h"
#include "utils/InterpolatingTreeMap.h"
#include "utils/TunableDouble.h"

class HardwareHoodIO : public HoodIO {
public:
  HardwareHoodIO(const int &servoHubId, const CANDevice &leftEncoder,
            const CANDevice &rightEncoder);

  void UpdateInputs(HoodIOInputs &inputs);
  void SetMicroseconds(double microseconds);
  void SetMicrosecondMap(std::vector<std::pair<double, int>>leftMap, std::vector<std::pair<double, int>>rightMap);
  void SetHoodPosition(units::angle::turn_t leftHoodPosition,
                       units::angle::turn_t rightHoodPosition);
  void SetHoodPosition(units::angle::turn_t hoodPosition);
private:
  rev::servohub::ServoHub m_servoHub;
  rev::servohub::ServoChannel m_leftServoChannel;
  rev::servohub::ServoChannel m_rightServoChannel;

  ctre::phoenix6::hardware::CANcoder m_leftEncoder;
  ctre::phoenix6::StatusSignal<units::turn_t> m_leftEncoderAngle;
  ctre::phoenix6::configs::CANcoderConfiguration m_leftEncoderConfigs;
  units::turn_t m_leftServoAngle{0_tr};
  int m_leftMicroseconds{1500};

  ctre::phoenix6::hardware::CANcoder m_rightEncoder;
  ctre::phoenix6::StatusSignal<units::turn_t> m_rightEncoderAngle;
  ctre::phoenix6::configs::CANcoderConfiguration m_rightEncoderConfigs;
  units::turn_t m_rightServoAngle{0_tr};
  int m_rightMicroseconds{1500};

  TunableDouble m_bigStep;
  TunableDouble m_littleStep;

  InterpolatingTreeMap<double, int> m_leftMicrosecondMap;
  InterpolatingTreeMap<double, int> m_rightMicrosecondMap;


  std::array<ctre::phoenix6::BaseStatusSignal *, 2> m_batchSignals;
};
