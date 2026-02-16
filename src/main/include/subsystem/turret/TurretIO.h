// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/torque.h>

struct TurretIOInputs {
  units::turn_t motorPosition{0_tr};
  units::turns_per_second_t motorVelocity{0_tps};
  units::radians_per_second_squared_t motorAcceleration{0_rad_per_s_sq};
  units::ampere_t motorCurrent{0_A};
  units::newton_meter_t motorTorque{0_Nm};

  units::second_t timestamp{0_s};
};

class TurretIO {
 public:
  virtual ~TurretIO() = default;

  virtual void UpdateInputs(TurretIOInputs& inputs) = 0;
  virtual void SetTurretAngle(units::radian_t angle) = 0;
};
