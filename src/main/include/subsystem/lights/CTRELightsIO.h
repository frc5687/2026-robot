// Team 5687 2026

#pragma once

#include <array>
#include <string>

#include "LightsIO.h"
#include "ctre/phoenix6/CANdle.hpp"
#include "ctre/phoenix6/controls/ControlRequest.hpp"
#include "ctre/phoenix6/controls/SolidColor.hpp"
#include "ctre/phoenix6/controls/StrobeAnimation.hpp"
#include "ctre/phoenix6/core/CoreCANdle.hpp"
#include "ctre/phoenix6/signals/RGBWColor.hpp"
#include "frc/DriverStation.h"
#include "frc/filter/Debouncer.h"
#include "subsystem/lights/LightState.h"
#include "units/frequency.h"
#include "units/time.h"
#include "utils/CANDevice.h"

class CTRELightsIO : public LightsIO {
public:
  CTRELightsIO(const CANDevice &candle);

  void UpdateInputs(LightsIOInputs &inputs) override;
  void SetSolidColor(LEDColor color) override;
  void SetBlinkColor(LEDColor color) override;
  LEDColor GetActiveColor() override;

private:
  ctre::phoenix6::hardware::CANdle m_candle;
  ctre::phoenix6::controls::SolidColor m_colorRequest;
  ctre::phoenix6::controls::StrobeAnimation m_colorBlink;
  ctre::phoenix6::signals::RGBWColor m_currentColor{0, 0, 0, 0};
  ctre::phoenix6::configs::LEDConfigs m_ledConfigs{};
  frc::DriverStation::Alliance m_alliance;
  frc::Debouncer m_debouncer;
  bool m_debouncedBlink;
  // scuffed lol
  bool m_blinkOn = true;
  static constexpr units::second_t kBlinkDuration{0.2};
};
