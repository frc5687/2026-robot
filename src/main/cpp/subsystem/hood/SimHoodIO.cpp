// #include "subsystem/hood/HoodIO.h"
// #include "subsystem/hood/SimHoodIO.h"
// #include "Constants.h"
// #include <frc/Timer.h>

// using namespace Constants::Hood;

// SimHoodIO::SimHoodIO()
//  : m_simHood(kMotor,
//     kGearRatio,
//     kMoi,
//     kArmLength,
//     kMinAngle,
//     kMaxAngle,
//     true,
//     0_rad),
//  m_pidController(Constants::Hood::SimPID::kP, Constants::Hood::SimPID::kI, Constants::Hood::SimPID::kD) {}


// void SimHoodIO::UpdateInputs(HoodIOInputs& inputs){
//     m_simHood.Update(20_ms);
//     inputs.timestamp = frc::Timer::GetFPGATimestamp();
//     inputs.leftHoodRotation = units::turn_t{m_simHood.GetAngle()};
//     inputs.rightHoodRotation = units::turn_t{m_simHood.GetAngle()};
// }


// void SimHoodIO::SetHoodPosition(units::angle::turn_t leftHoodRotation){
//     auto pidOutput = m_pidController.Calculate(leftHoodRotation.value());
//     m_simHood.SetInputVoltage(units::volt_t{pidOutput});
// }
