#include "frc/simulation/SingleJointedArmSim.h"
#include "subsystem/shooter/hood/HoodIO.h"
#include "subsystem/shooter/hood/SimHoodIO.h"
#include "subsystem/shooter/hood/HoodConstants.h"
#include <frc./Timer.h>

using namespace Constants::Hood;

SimHoodIO::SimHoodIO()
 : m_simHood(kMotor, kGearRatio, kMoi, kArmLength, kMinAngle, kMaxAngle, true, 0_rad),
 m_pidController(10, 0, 0) {}


void UpdateInputs(HoodIOInputs& inputs){
    m_simHood.Update(20_ms);

}


void SetHoodPosition(units::angle::turn_t hoodRotation){

}
