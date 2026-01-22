#include "subsystem/flywheel/SimFlywheelIO.h"
#include "frc/Timer.h"
#include "frc/system/plant/DCMotor.h"
#include "subsystem/flywheel/FlywheelIO.h"
#include "units/moment_of_inertia.h"
#include <frc/system/plant/LinearSystemId.h>
#include "subsystem/flywheel/FlywheelConstants.h"
#include "units/voltage.h"

using namespace Constants::Flywheel;

SimFlywheelIO::SimFlywheelIO() : m_flywheelSim(
  frc::LinearSystemId::FlywheelSystem(kMotor, 0.02_kg_sq_m, kGearRatio),
  kMotor,
  {0.01}), m_controller(kP, kI, kD){}

void SimFlywheelIO::UpdateInputs(FlywheelIOInputs& inputs) {
  constexpr auto kDt = 20_ms;
  m_flywheelSim.Update(kDt);
  
  inputs.flywheelVelocity = m_flywheelSim.GetAngularVelocity();
  inputs.motorVelocity = m_flywheelSim.GetAngularVelocity() / kGearRatio;
  inputs.timestamp = frc::Timer::GetFPGATimestamp();

  auto output = m_controller.Calculate(m_flywheelSim.GetAngularVelocity().value());
  m_flywheelSim.SetInputVoltage(units::volt_t{output});
}

void SimFlywheelIO::SetFlywheelRPM(units::revolutions_per_minute_t desiredRPM) {
  m_controller.SetSetpoint(desiredRPM.value());
}
