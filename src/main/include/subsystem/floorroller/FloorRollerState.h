// Team 5687 2026

#pragma once

#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>

#include "subsystem/floorroller/FloorRollerIO.h"

struct FloorRollerState {
  units::turns_per_second_t velocity{0_tps};
  units::ampere_t statorCurrent{0_A};
  units::second_t timestamp{0_s};

  units::turns_per_second_t prevVelocity{0_tps};
  units::ampere_t prevStatorCurrent{0_A};

  units::ampere_t currentDelta{0_A};
  units::turns_per_second_t velocityDelta{0_tps};

  bool ballDetected{false};
  bool ballSeated{false};
  bool ballFed{false};

  units::second_t detectionTimestamp{0_s};
  bool wasDetected{false};

  void Update(const FloorRollerIOInputs &inputs,
              units::ampere_t currentDeltaThreshold,
              units::turns_per_second_t velocityDipThreshold,
              units::ampere_t seatedCurrentCeiling,
              units::second_t debounceTime) {
    prevVelocity = velocity;
    prevStatorCurrent = statorCurrent;

    velocity = inputs.motorVelocity;
    statorCurrent = inputs.statorCurrent;
    timestamp = inputs.timestamp;

    currentDelta = statorCurrent - prevStatorCurrent;
    velocityDelta = velocity - prevVelocity;

    bool currentRising = currentDelta > currentDeltaThreshold;
    bool decelerating = velocityDelta < -velocityDipThreshold;

    if (currentRising && decelerating) {
      ballDetected = true;
      detectionTimestamp = timestamp;
    } else if (ballDetected) {
      auto elapsed = timestamp - detectionTimestamp;
      if (elapsed > debounceTime) {
        ballDetected = false;
      }
    }

    ballSeated =
        wasDetected && !ballDetected && statorCurrent < seatedCurrentCeiling;

    ballFed = wasDetected && prevStatorCurrent > seatedCurrentCeiling &&
              statorCurrent < seatedCurrentCeiling;

    wasDetected = ballDetected || ballSeated;
  }
};
