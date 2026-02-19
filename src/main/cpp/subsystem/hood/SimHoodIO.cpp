// Team 5687 2026

#include "subsystem/hood/SimHoodIO.h"

#include <frc/Timer.h>

#include "Constants.h"
#include "subsystem/hood/HoodIO.h"

using namespace Constants::Hood;

SimHoodIO::SimHoodIO()
    : m_simHood(kMotor, kGearRatio, kMoi, kArmLength, kMinAngle, kMaxAngle,
                true, 0_rad),
      m_pidController(Constants::Hood::SimPID::kP, Constants::Hood::SimPID::kI,
                      Constants::Hood::SimPID::kD) {}

void SimHoodIO::UpdateInputs(HoodIOInputs &inputs) {
  inputs.timestamp = frc::Timer::GetFPGATimestamp();

  auto pidOutput = m_pidController.Calculate(m_desiredRotation.value());
  m_simHood.SetInputVoltage(units::volt_t{pidOutput});

  m_simHood.Update(20_ms);
  auto hoodPosition = m_simHood.GetAngle();
  auto hoodVelocity = m_simHood.GetVelocity();

  // we need to convert to motor rotations to simulate what ctre does
  inputs.leftHoodRotation =
      (hoodPosition * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;

  inputs.rightHoodRotation =
      (hoodPosition * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;

  inputs.leftHoodVelocity =
      (hoodVelocity * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;

  inputs.rightHoodVelocity =
      (hoodVelocity * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;
}

void SimHoodIO::SetHoodPosition(units::angle::turn_t hoodRotation) {
  m_desiredRotation = hoodRotation;
}

void SimHoodIO::SetHoodPosition(units::turn_t leftHoodPosition,
                                units::turn_t rightHoodPosition) {
  SetHoodPosition(leftHoodPosition);
}
