#pragma once

#include "FlywheelIO.h"
#include "frc/controller/SimpleMotorFeedforward.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "utils/TunableDouble.h"
#include <frc/simulation/FlywheelSim.h>
#include <frc/controller/PIDController.h>
#include <frc/filter/LinearFilter.h>

class SimFlywheelIO : public FlywheelIO {
public:
  SimFlywheelIO();
  ~SimFlywheelIO() = default;

  void UpdateInputs(FlywheelIOInputs& inputs) override;
  void SetFlywheelRPM(units::revolutions_per_minute_t desiredRPM, units::revolutions_per_minute_t desiredRPMfake) override;

private:
  frc::sim::FlywheelSim m_flywheelSim;
  frc::PIDController m_controller;
  frc::SimpleMotorFeedforward<units::radians> m_feedForward;

  TunableDouble m_kP;
  TunableDouble m_kI;
  TunableDouble m_kD;

  TunableDouble m_kS;
  TunableDouble m_kV;
  TunableDouble m_kA;

  units::revolutions_per_minute_t m_desiredRPM{0_rpm};
};
