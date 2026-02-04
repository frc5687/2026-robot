#include "frc/system/plant/LinearSystemId.h"
#include "subsystem/intake/SimIntakeRollerIO.h"

#include "frc/Timer.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "subsystem/intake/linearintake/LinearIntake.h"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"
#include "units/angle.h"
#include "units/length.h"
#include <iostream>
#include <numbers>
#include "subsystem/intake/linearintake/SimLinearIntakeIO.h"

using namespace Constants::LinearIntake;

SimLinearIntakeIO::SimLinearIntakeIO()
    : m_linearIntakeSim(kMotor,
                    kGearRatio, kMass,
                    kDrumRadius,
                    kMinExtension,
                    kMaxExtension, true, 0_m, {0.001, 0.001}),
      m_pidController(100, 0, 0,
                      frc::TrapezoidProfile<units::meter>::Constraints(
                          kMaxVelocity,
                          kMaxAccel)) {}

void SimLinearIntakeIO::UpdateInputs(LinearIntakeIOInputs& inputs) {
  m_linearIntakeSim.Update(20_ms);
  inputs.linearIntakePosition = m_linearIntakeSim.GetPosition();
  inputs.linearIntakeVelocity = m_linearIntakeSim.GetVelocity();
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimLinearIntakeIO::SetPosition(units::meter_t desiredMeters) {
 auto position = m_linearIntakeSim.GetPosition();
  auto pidOutput = m_pidController.Calculate(position, desiredMeters);

  m_linearIntakeSim.SetInputVoltage(units::volt_t{pidOutput});
}