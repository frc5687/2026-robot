#include "subsystem/intake/SimIntakeRollerIO.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "frc/Timer.h"
#include "frc/system/plant/LinearSystemId.h"
#include "subsystem/intake/IntakeRoller.h"
#include <units/voltage.h>



using namespace Constants::IntakeRoller;
 
SimIntakeRollerIO::SimIntakeRollerIO()
     : m_IntakeRollerSim(frc::LinearSystemId::DCMotorSystem(kMotor, kMass, kMotorGearRatio), kMotor, {0.001,0.001}) {}
    // m_pidController(kP, kI, kD)
   


void SimIntakeRollerIO::UpdateInputs(IntakeRollerIOInputs& inputs) {
   m_IntakeRollerSim.Update(20_ms);
   inputs.IntakeRollerPosition = m_IntakeRollerSim.GetAngularPosition();
   inputs.IntakeRollerVelocity = m_IntakeRollerSim.GetAngularVelocity();
   inputs.voltage = m_IntakeRollerSim.GetInputVoltage();
   inputs.timestamp = frc::Timer::GetFPGATimestamp();
 }
void SimIntakeRollerIO::SetVoltage(units::volt_t voltage) {
     m_IntakeRollerSim.SetInputVoltage(voltage);
}
//void SimIntakeRollerIO::SetIntakeRPM(units::angular_velocity::radians_per_second_t DesiredAngularVelocity) {
//auto AngularVelocity = m_IntakeRollerSim.GetAngularVelocity();
//auto pidOutput = m_pidController.Calculate(AngularVelocity.value(), DesiredAngularVelocity.value());
 //m_IntakeRollerSim.SetInputVoltage(units::volt_t{pidOutput});
//}