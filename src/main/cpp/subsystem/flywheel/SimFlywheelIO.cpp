// Team 5687 2026

#include "subsystem/flywheel/SimFlywheelIO.h"

#include <frc/system/plant/LinearSystemId.h>

#include "Constants.h"
#include "subsystem/flywheel/FlywheelIO.h"

using namespace Constants::Flywheel;

SimFlywheelIO::SimFlywheelIO()
    : m_flywheelSim(
          frc::LinearSystemId::FlywheelSystem(kMotor, kInertia, kGearRatio),
          kMotor),
      m_controller(kP, kI, kD) {}

void SimFlywheelIO::UpdateInputs(FlywheelIOInputs& inputs) {
  constexpr auto kDt = 20_ms;
  m_flywheelSim.Update(kDt);
  inputs.motorVelocity = m_flywheelSim.GetAngularVelocity() * kGearRatio;
  inputs.motorAcceleration =
      m_flywheelSim.GetAngularAcceleration() * kGearRatio;
}
