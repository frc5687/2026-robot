#include "frc/system/plant/LinearSystemId.h"
#include "subsystem/intake/SimIntakeRollerIO.h"

#include "frc/Timer.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "subsystem/intake/linearintake/LinearIntake.h"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"
#include "subsystem/intake/linearintake/SimLinearIntakeIO.h"

using namespace Constants::LinearIntake;

SimLinearIntakeIO::SimLinearIntakeIO()
    : m_linearIntakeSim(frc::LinearSystemId::DCMotorSystem(kMotor, kInertia, kGearRatio), kMotor, {0.001,0.001}),
      m_pidController(kP, kI, kD) {
}

void SimLinearIntakeIO::UpdateInputs(LinearIntakeIOInputs& inputs) {
  m_linearIntakeSim.Update(20_ms);
  inputs.linearIntakePosition = m_linearIntakeSim.GetAngularPosition().value() * kCircumference / kGearRatio;
  inputs.linearIntakeVelocity = m_linearIntakeSim.GetAngularVelocity();
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimLinearIntakeIO::SetPosition(units::meter_t desiredMeters) {
  auto position = m_linearIntakeSim.GetAngularPosition().value() / kCircumference.value() * kGearRatio;
  auto pidOutput = m_pidController.Calculate(position);

  m_linearIntakeSim.SetInputVoltage(units::volt_t{pidOutput});
}