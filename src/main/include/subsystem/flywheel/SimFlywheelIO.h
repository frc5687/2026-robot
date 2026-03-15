// Team 5687 2026

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include "subsystem/flywheel/FlywheelConstants.h"
#include "FlywheelIO.h"

class SimFlywheelIO : public FlywheelIO {
public:
  SimFlywheelIO();
  ~SimFlywheelIO() override = default;

  void UpdateInputs(FlywheelIOInputs &inputs) override;
  void SetMotorVelocity(units::turns_per_second_t rps) override;
  void SetVoltage(units::volt_t voltage) override;

private:
  struct SimMotor {
    frc::sim::FlywheelSim sim;
    frc::PIDController pid;
    frc::SimpleMotorFeedforward<units::radians> feedforward;

    units::turns_per_second_t setpoint{0_tps};
    units::turn_t motorPosition{0_tr};
    units::volt_t lastAppliedVoltage{0_V};
    double gearing;

    SimMotor(const frc::LinearSystem<1, 1, 1> &plant, const frc::DCMotor &motor,
             double gearRatio, double kP, double kD,
             decltype(1_V / 1_rad_per_s) kVff, units::volt_t kSff,
             decltype(1_V / 1_rad_per_s_sq) kAff);

    units::volt_t CalculateClosedLoop();
    void Update(units::second_t dt, units::volt_t voltage);
    units::radians_per_second_t GetMechanismVelocity() const;
    units::turns_per_second_t GetMotorVelocity() const;
    units::ampere_t GetCurrentDraw() const;
  };

  SimMotor m_flywheel; // single mechanism

  bool m_characterizing{false};
  units::volt_t m_characterizationVoltage{0_V};

  static SimMotor MakeSimMotor();
};
