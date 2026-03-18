
// Team 5687 2026


#pragma once


#include <string>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>
#include "LightState.h"
#include "ctre/phoenix6/signals/RGBWColor.hpp"
#include "subsystem/lights/LightState.h"




struct LightsIOInputs {
};


class LightsIO {
public:
  virtual ~LightsIO() = default;


  virtual void UpdateInputs(LightsIOInputs &inputs) = 0;
  virtual void SetSolidColor(LEDColor color) = 0;
  virtual void SetBlinkColor(LEDColor color) = 0;
  virtual LEDColor GetActiveColor() = 0;
};


