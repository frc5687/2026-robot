#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include<units/angular_velocity.h>
struct LinearIntakeIOInputs{

    units::turn_t motorPosition{0.0_tr};
    units::turns_per_second_t motorVelocity{0.0_tps};

    units::meter_t linearIntakePosition{0.0_m};
    units::meters_per_second_t linearIntakeVelocity{0_mps};

    units::second_t timestamp{0_s};
};

class LinearIntakeIO{
    public:
        virtual ~LinearIntakeIO() = default;
        virtual void UpdateInputs(LinearIntakeIOInputs& inputs) = 0;
        virtual void SetPosition(units::meter_t meters) = 0;
};