#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include<units/angular_velocity.h>
struct LinearIntakeIOInputs{

    units::meter_t linearIntakePosition{0_m};
    units::angular_velocity::radians_per_second_t linearIntakeVelocity{0_rad_per_s};

    units::second_t timestamp{0_s};
};

class LinearIntakeIO{
    public:
        virtual ~LinearIntakeIO() = default;
        virtual void UpdateInputs(LinearIntakeIOInputs& inputs) = 0;
        virtual void SetPosition(units::meter_t meters) = 0;
};