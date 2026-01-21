
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <iostream>

struct IndexerIOInputs {
  units::turn_t MotorPosition{0_tr};
  units::turns_per_second_t MotorVelocity{0_tps};

  units::second_t timestamp{0_s};
};

class IndexerIO {
 public:
  virtual ~IndexerIO() = default;
  virtual void UpdateInputs(IndexerIOInputs& inputs) = 0;
  virtual void SetMotorVoltage(units::volt_t voltage) {std::cout<<"Didn't make motor voltage function";}
};