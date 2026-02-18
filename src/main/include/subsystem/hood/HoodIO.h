#pragma once

#include <units/angle.h>
#include <units/time.h>

struct HoodIOInputs{
    units::second_t timestamp{0_s};
    int leftMicroseconds = 0;
    units::angle::turn_t leftHoodRotation{0_tr};
    int rightMicroseconds = 0;
    units::angle::turn_t rightHoodRotation{0_tr};
};

class HoodIO {
public:
    virtual ~HoodIO() = default;
    virtual void UpdateInputs(HoodIOInputs& inputs) = 0;
    virtual void SetHoodPosition(units::turn_t leftHoodPosition, units::turn_t rightHoodPosition) = 0;
};
