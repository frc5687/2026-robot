#include "CTREHoodI.h"

CTREHoodIO::CTREHoodIO(const CANDevice& hoodmotor)
    : m_hoodMotor(hoodmotor.id, hoodmotor.bus);


CTREHoodIO::SetHoodAngle(units::radians angle)
{
}
