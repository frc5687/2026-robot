#include "frc/simulation/SingleJointedArmSim.h"
#include "subsystem/shooter/hood/HoodIO.h"
#include "subsystem/shooter/hood/SimHoodIO.h"
#include "subsystem/shooter/hood/HoodConstants.h"
#include <frc/Timer.h>

using namespace Constants::Hood;

SimHoodIO::SimHoodIO()
 : m_simHood(kMotor, 
    kGearRatio,
    kMoi, 
    kArmLength, 
    kMinAngle, 
    kMaxAngle, 
    true, 
    0_rad),
 m_pidController(10, 0, 0) {}


void SimHoodIO::UpdateInputs(HoodIOInputs& inputs){
    m_simHood.Update(20_ms);
    inputs.hoodRotation = units::turn_t{m_simHood.GetAngle()};
}


void SimHoodIO::SetHoodPosition(units::angle::turn_t hoodRotation){
    auto pidOutput = m_pidController.Calculate(hoodRotation.value());

    m_simHood.SetInputVoltage(units::volt_t{pidOutput});
} 
