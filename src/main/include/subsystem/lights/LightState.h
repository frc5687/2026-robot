#pragma once


#include "ctre/phoenix6/signals/RGBWColor.hpp"


enum class LEDColor {
    Purple,
    Red,
    Green,
    Blue,
    Yellow,
    Off
};


inline ctre::phoenix6::signals::RGBWColor ToRGBW(LEDColor color) {
    switch (color) {
        case LEDColor::Purple:   return {200, 0,   200,   0};
        case LEDColor::Red:   return {255, 0,   0,   0};
        case LEDColor::Green: return {0,   255, 0,   0};
        case LEDColor::Blue:  return {0,   0,   255, 0};
        case LEDColor::Yellow: return {255,   220,   0,   0};
        case LEDColor::Off:   return {0,   0,   0,   0};
        default:              return {0,   0,   0,   0};
    }
}
