#pragma once

#include <units/angle.h>
#include <units/time.h>
#include <units/angular_velocity.h>

struct HoodIOInputs{
    units::second_t timestamp{0_s};
    int leftMicroseconds = 0;
    units::angle::turn_t leftHoodRotation{0_tr};
    units::turns_per_second_t leftHoodVelocity{0_tps};

    int rightMicroseconds = 0; // TODO: units::micro_seconds
    units::angle::turn_t rightHoodRotation{0_tr};
    units::turns_per_second_t rightHoodVelocity{0_tps};

};

class HoodIO {
public:
    virtual ~HoodIO() = default;
    virtual void UpdateInputs(HoodIOInputs& inputs) = 0;
    virtual void SetHoodPosition(units::turn_t hoodPosition) = 0;
    virtual void SetHoodPosition(units::turn_t leftHoodPosition, units::turn_t rightHoodPosition) = 0;
};
