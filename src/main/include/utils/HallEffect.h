// Team 5687 2026
#pragma once

#include <frc/DigitalInput.h>

class HallEffect : public frc::DigitalInput {
public:
  explicit HallEffect(int channel) : frc::DigitalInput(channel) {}
  ~HallEffect() override = default;

  // Returns true when the Hall Effect sensor detects a magnet.
  // Inverts the raw digital signal since the sensor is active-low.
  bool Get() const { return !frc::DigitalInput::Get(); }
};
