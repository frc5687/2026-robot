// #include "subsystem/intake/SimIntakeRollerIO.h"

// #include "frc/Timer.h"
// #include "frc/trajectory/TrapezoidProfile.h"
// #include "subsystem/intake/IntakeRoller.h"

// SimIntakeRollerIO::SimIntakeRollerIO()
//     : m_IntakeRollerSim(),
//       m_pidController(100, 0, 0,
//                       frc::TrapezoidProfile<units::meter>::Constraints(
//                           Constants::IntakeRoller::kMaxVelocity,
//                           Constants::IntakeRoller::kMaxAccel)) {}

// void SimElevatorIO::UpdateInputs(ElevatorIOInputs& inputs) {
//   m_elevatorSim.Update(20_ms);
//   inputs.elevatorPosition = m_elevatorSim.GetPosition();
//   inputs.elevatorVelocity = m_elevatorSim.GetVelocity();
//   inputs.timestamp = frc::Timer::GetFPGATimestamp();
// }

// void SimElevatorIO::SetElevatorHeight(units::meter_t desiredHeight) {
//   auto position = m_elevatorSim.GetPosition();
//   auto pidOutput = m_pidController.Calculate(position, desiredHeight);

//   m_elevatorSim.SetInputVoltage(units::volt_t{pidOutput});
// }