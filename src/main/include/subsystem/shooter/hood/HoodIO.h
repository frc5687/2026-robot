#pragma once

#include <units/angle.h>

struct HoodIOInputs{

    units::angle::turn_t hoodRotation{0_tr};
    

};

class HoodIO {
    public:
        virtual ~HoodIO() = default;
        virtual void UpdateInputs(HoodIOInputs& inputs) = 0;
        virtual void SetHoodPosition(units::turn_t hoodPosition) = 0;

};