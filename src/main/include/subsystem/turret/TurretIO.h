#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

struct TurretIOInputs {
    units::turn_t motorPosition{0_tr};
    units::turns_per_second_t motorVelocity{0_tps};

    units::radian_t angle{0_rad};
    units::radians_per_second_t angularVelocity{0_rad_per_s};

    units::second_t timestamp{0_s};
};

class TurretIO {
public:
    virtual ~TurretIO() = default;

    virtual void UpdateInputs(TurretIOInputs& inputs) = 0;
    virtual void SetTurretAngle(units::radian_t angle) = 0;
};
