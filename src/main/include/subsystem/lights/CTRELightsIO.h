// Team 5687 2026

#pragma once

#include <array>

#include <string>
#include "Constants.h"
#include "LightsIO.h"
#include "ctre/phoenix6/controls/ControlRequest.hpp"
#include "ctre/phoenix6/controls/SolidColor.hpp"
#include "ctre/phoenix6/signals/RGBWColor.hpp"
#include "utils/CANDevice.h"
#include "ctre/phoenix6/CANdle.hpp"
#include "subsystem/lights/LightState.h"


class CTRELightsIO : public LightsIO {
public:
  CTRELightsIO(const CANDevice &candle);

  void UpdateInputs(LightsIOInputs &inputs) override;
  void SetColor(LEDColor color) override;

 private:
  ctre::phoenix6::hardware::CANdle m_candle;
  ctre::phoenix6::controls::SolidColor m_colorRequest;
  ctre::phoenix6::signals::RGBWColor m_currentColor{0, 0, 0, 0};
};
