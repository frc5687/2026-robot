#include "subsystem/intake/IntakeRoller.h"

#include "subsystem/LoggedSubsystem.h"
#include <iostream>
#include "subsystem/intake/IntakeRollerIO.h"

IntakeRoller::IntakeRoller(std::unique_ptr<IntakeRollerIO> io) 
    : LoggedSubsystem("IntakeRoller"), m_io(std::move(io)) {}

void IntakeRoller::UpdateInputs() {m_io->UpdateInputs(m_inputs);}

//void IntakeRoller::SetVoltage(units::volt_t voltage) {
 //   std::cout << "Setting Voltage\n";
 //   m_io->SetVoltage(voltage);
//}

void IntakeRoller::SetIntakeRPM(units::radians_per_second_t rads){
    m_io->SetIntakeRPM(rads);
}

void IntakeRoller::LogTelemetry(){
    Log("intake velocity", m_inputs.IntakeRollerVelocity.value());
    Log("voltage", m_inputs.voltage.value());
}
    