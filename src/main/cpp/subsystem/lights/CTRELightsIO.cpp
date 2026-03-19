// Team 5687 2026

#include "subsystem/lights/CTRELightsIO.h"

#include <frc/Timer.h>
#include <string>

#include "RobotState.h"
#include "ctre/phoenix6/CANdle.hpp"
#include "ctre/phoenix6/controls/StaticBrake.hpp"
#include "ctre/phoenix6/signals/RGBWColor.hpp"
#include "subsystem/lights/LightState.h"
CTRELightsIO::CTRELightsIO(const CANDevice &candle):
m_candle(candle.id, candle.bus),
m_colorRequest(0, 200){
}

void CTRELightsIO::UpdateInputs(LightsIOInputs &inputs) {
    SetColor(LEDColor::Green); 
}

void CTRELightsIO::SetColor(LEDColor color) {   
    m_currentColor = ToRGBW(color);
    m_candle.SetControl(m_colorRequest.WithColor(m_currentColor));
}

