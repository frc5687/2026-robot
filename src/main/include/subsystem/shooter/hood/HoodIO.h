#pragma once


struct HoodIOInputs{

};

class HoodIO {
    public:
    virtual ~HoodIO() = default;

    virtual void UpdateInputs(HoodIOInputs& inputs) = 0;
    virtual void SetHoodAngle(double angle);

};