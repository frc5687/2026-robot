#include "subsystem/flywheel/SimFlywheelIO.h"
#include "frc/Timer.h"
#include "frc/system/plant/DCMotor.h"
#include "subsystem/flywheel/FlywheelIO.h"
#include "units/angular_velocity.h"
#include "units/moment_of_inertia.h"
#include <frc/system/plant/LinearSystemId.h>
#include "subsystem/flywheel/FlywheelConstants.h"
#include "units/voltage.h"

using namespace Constants::Flywheel;

SimFlywheelIO::SimFlywheelIO() : m_flywheelSim(
  frc::LinearSystemId::FlywheelSystem(kMotor, kInertia, kGearRatio),
  kMotor,
  {0.01}), m_controller(kP, kI, kD),
  m_feedForward(
    units::volt_t{kS},
    units::volt_t{kV} / 1_rpm,
    units::volt_t{kA} / 1_rev_per_m_per_s,
    20_ms),
  m_kP("Flywheel", "kP", kP),
  m_kI("Flywheel", "kI", kI),
  m_kD("Flywheel", "kD", kD),
  m_kS("Flywheel", "kS", kS),
  m_kV("Flywheel", "kV", kV),
  m_kA("Flywheel", "kA", kA){}

void SimFlywheelIO::UpdateInputs(FlywheelIOInputs& inputs) {
  if (m_kP.HasChanged() || m_kI.HasChanged() || m_kD.HasChanged()) {
    m_controller.SetPID(m_kP.Get(), m_kI.Get(), m_kD.Get());
  }

  if (m_kS.HasChanged() || m_kV.HasChanged() || m_kA.HasChanged()) {
    m_feedForward.SetKs(units::volt_t{m_kS.Get()});
    m_feedForward.SetKv(units::volt_t{m_kV.Get()} / 1_rpm);
    m_feedForward.SetKa(units::volt_t{m_kA.Get()} / 1_rev_per_m_per_s);
  }

  units::revolutions_per_minute_t currentVelocity = m_flywheelSim.GetAngularVelocity();
  auto pidOutput = m_controller.Calculate(currentVelocity.value());
  auto ffOutput = m_feedForward.Calculate(m_desiredRPM);
  m_flywheelSim.SetInputVoltage(units::volt_t{pidOutput} + units::volt_t{ffOutput});

  constexpr auto kDt = 20_ms;
  m_flywheelSim.Update(kDt);

  inputs.flywheelVelocity = m_flywheelSim.GetAngularVelocity();
  inputs.motorVelocity = m_flywheelSim.GetAngularVelocity() / kGearRatio;
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimFlywheelIO::SetFlywheelRPM(units::revolutions_per_minute_t desiredRPM) {
  m_controller.SetSetpoint(desiredRPM.value());
  m_desiredRPM = desiredRPM;
}
