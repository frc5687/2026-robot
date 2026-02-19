// Team 5687 2026

//
// IndexerState — Ball detection state machine.
//
// Detection approach:
//   Raw current thresholds and velocity dips are noisy and produce
//   false positives.  Instead we use:
//
//   1. CURRENT DERIVATIVE (dI/dt): A ball entering the indexer causes
//      a sharp current spike — the rate of change matters more than
//      the absolute value.  A motor under constant load has high
//      current but low dI/dt.  A ball impact has high dI/dt.
//
//   2. VELOCITY DIP (dω/dt): The motor decelerates when a ball
//      loads it.  Combining with current derivative eliminates
//      false triggers from voltage transients.
//
//   3. DEBOUNCE: Once detected, hold the detection for a minimum
//      duration to prevent flicker.
//
// Ball lifecycle:
//   BallDetected → true when ball first contacts rollers
//   BallSeated   → true when ball is settled (current spike ended,
//                   velocity recovered, ball is sitting in the indexer)
//   BallFed      → true on falling edge (ball has exited toward flywheel)
#pragma once

#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>

#include "IndexerIO.h"

struct IndexerState {
  // ── Current measurements ─────────────────────────────────────────────
  units::turns_per_second_t velocity{0_tps};
  units::ampere_t statorCurrent{0_A};
  units::second_t timestamp{0_s};

  // ── Previous cycle ───────────────────────────────────────────────────
  units::turns_per_second_t prevVelocity{0_tps};
  units::ampere_t prevStatorCurrent{0_A};

  // ── Derived signals ──────────────────────────────────────────────────
  units::ampere_t currentDelta{0_A}; // dI per cycle
  units::turns_per_second_t velocityDelta{
      0_tps}; // dω per cycle (negative = decel)

  // ── Detection outputs ────────────────────────────────────────────────
  bool ballDetected{false}; // Ball is actively loading the indexer
  bool ballSeated{false};   // Ball is settled in position (ready to feed)
  bool ballFed{false};      // Ball just exited (falling edge, one-shot)

  // ── Debounce ─────────────────────────────────────────────────────────
  units::second_t detectionTimestamp{0_s};
  bool wasDetected{false};

  void Update(const IndexerIOInputs &inputs,
              units::ampere_t currentDeltaThreshold,
              units::turns_per_second_t velocityDipThreshold,
              units::ampere_t seatedCurrentCeiling,
              units::second_t debounceTime) {
    // ── Shift history ────────────────────────────────────────────────
    prevVelocity = velocity;
    prevStatorCurrent = statorCurrent;

    velocity = inputs.motorVelocity;
    statorCurrent = inputs.statorCurrent;
    timestamp = inputs.timestamp;

    // ── Compute deltas ───────────────────────────────────────────────
    // Positive currentDelta = current increasing (load applied)
    // Negative velocityDelta = motor decelerating (ball loading)
    currentDelta = statorCurrent - prevStatorCurrent;
    velocityDelta = velocity - prevVelocity;

    // ── Ball detection: current rising fast AND velocity dropping ─────
    bool currentRising = currentDelta > currentDeltaThreshold;
    bool decelerating = velocityDelta < -velocityDipThreshold;

    if (currentRising && decelerating) {
      ballDetected = true;
      detectionTimestamp = timestamp;
    } else if (ballDetected) {
      // Hold detection for debounce duration
      auto elapsed = timestamp - detectionTimestamp;
      if (elapsed > debounceTime) {
        ballDetected = false;
      }
    }

    // ── Ball seated: was detected, current has settled back down ──────
    // The ball is in position, no longer actively loading the motor.
    ballSeated =
        wasDetected && !ballDetected && statorCurrent < seatedCurrentCeiling;

    // ── Ball fed: was seated/detected, now current dropped sharply ────
    // One-shot falling edge: ball just left the indexer.
    ballFed = wasDetected && prevStatorCurrent > seatedCurrentCeiling &&
              statorCurrent < seatedCurrentCeiling;

    wasDetected = ballDetected || ballSeated;
  }
};
