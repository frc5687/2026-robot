

#include "subsystem/lights/CTRELightsIO.h"


#include <frc/Timer.h>

#include "ctre/phoenix6/CANdle.hpp"
#include "ctre/phoenix6/configs/LEDConfigs.hpp"
#include "ctre/phoenix6/signals/RGBWColor.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/DriverStation.h"
#include "subsystem/lights/LightState.h"

#include "utils/MatchTracker.h"

CTRELightsIO::CTRELightsIO(const CANDevice &candle):
m_candle(candle.id, candle.bus),
m_colorRequest(0, 355),
m_colorBlink(0, 355),
m_alliance(frc::DriverStation::kRed),
m_debouncer(0.1_s){
m_ledConfigs.WithStripType(ctre::phoenix6::signals::StripTypeValue::BRG);
m_candle.GetConfigurator().Apply(m_ledConfigs);
}


void CTRELightsIO::UpdateInputs(LightsIOInputs &inputs) {
    m_debouncedBlink = m_debouncer.Calculate(m_blinkOn);
    m_alliance = frc::DriverStation::GetAlliance().value();
    m_debouncedBlink ? SetBlinkColor(GetActiveColor()) : SetSolidColor(GetActiveColor());
}


LEDColor CTRELightsIO::GetActiveColor(){
   
    if (MatchTracker::Instance().CanAllianceScore(m_alliance)) {
    m_blinkOn = false;
    return LEDColor::Green;
} else if (MatchTracker::Instance().ShouldPrespinForAlliance(m_alliance, 3_s)) {
    m_blinkOn = true;
    return LEDColor::Green;
} else if (MatchTracker::Instance().ShouldPrespinForAlliance(m_alliance, 7_s)) {
    m_blinkOn = true;
    return LEDColor::Yellow;
} else if (MatchTracker::Instance().ShouldPrespinForAlliance(m_alliance, 15_s)) {
    m_blinkOn = false;
    return LEDColor::Yellow;
} else {
    m_blinkOn = false;
    return frc::DriverStation::kBlue == m_alliance ? LEDColor::Blue : LEDColor::Red;
}


}


void CTRELightsIO::SetSolidColor(LEDColor color) {  
    m_currentColor = ToRGBW(color);
    m_candle.ClearAllAnimations();
    m_candle.SetControl(m_colorRequest.WithColor(m_currentColor));
}


void CTRELightsIO::SetBlinkColor(LEDColor color) {  
    m_currentColor = ToRGBW(color);
        m_candle.SetControl(m_colorBlink.WithColor(m_currentColor).WithFrameRate(units::hertz_t{5}));
}




