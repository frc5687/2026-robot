// Team 5687 2026
#include "subsystem/flywheel/SimFlywheelIO.h"

#include <frc/Timer.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Constants.h"
#include "subsystem/flywheel/FlywheelIO.h"

using namespace Constants::Flywheel;

SimFlywheelIO::SimFlywheelIO()
    : m_flywheelSim(
          frc::LinearSystemId::FlywheelSystem(kMotor, kInertia, kGearRatio),
          kMotor, {0.001}),
      m_controller(kP, kI, kD) {}

void SimFlywheelIO::SetFlywheelRPM (units::revolutions_per_minute_t desiredRPM) {}

void SimFlywheelIO::UpdateInputs(FlywheelIOInputs& inputs) {
  constexpr auto kDt = 20_ms;

  // Sim gives output-shaft rad/s; multiply by gear ratio → motor-shaft.
  // Library implicitly converts rad/s → turns/s on assignment.
  units::turns_per_second_t currentMotorVelocity =
      m_flywheelSim.GetAngularVelocity() * kGearRatio;

  // m_desiredRPM is motor-shaft RPM; library converts RPM → turns/s.
  units::turns_per_second_t target{m_desiredRPM};

  units::volt_t voltage =  units::volt_t{m_controller.Calculate(currentMotorVelocity.value(), target.value())};

  m_flywheelSim.SetInputVoltage(voltage);
  m_flywheelSim.Update(kDt);

  // Populate IO inputs as motor-shaft quantities.
  // GetAngularVelocity() is output-shaft rad/s * gear ratio → motor-shaft.
  // Assigned to turns_per_second_t; library handles rad → turns.
  inputs.motorVelocity = m_flywheelSim.GetAngularVelocity() * kGearRatio;
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimFlywheelIO::SetMotorRPM(units::revolutions_per_minute_t motorRPM) {
  m_desiredRPM = motorRPM;
}
