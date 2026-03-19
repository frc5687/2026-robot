#pragma once

#include "ctre/phoenix6/signals/RGBWColor.hpp"

enum class LEDColor {
    Red,
    Green,
    Blue,
    White,
    Off
};

inline ctre::phoenix6::signals::RGBWColor ToRGBW(LEDColor color) {
    switch (color) {
        case LEDColor::Red:   return {255, 0,   0,   0};
        case LEDColor::Green: return {0,   255, 0,   255};
        case LEDColor::Blue:  return {0,   0,   255, 0};
        case LEDColor::White: return {0,   0,   0,   255};
        case LEDColor::Off:   return {0,   0,   0,   0};
        default:              return {0,   0,   0,   0};
    }
}