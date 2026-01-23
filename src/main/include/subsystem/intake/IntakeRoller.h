
#pragma once
#include <units/length.h>
#include <units/mass.h>
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>

#include <numbers>

#include "frc/system/plant/DCMotor.h"
#include "units/angular_acceleration.h"

#include "subsystem/intake/IntakeRollerIO.h"
#include "subsystem/LoggedSubsystem.h"
#include <units/angle.h>
#include <frc/trajectory/TrapezoidProfile.h>



namespace Constants::IntakeRoller {

    inline constexpr bool kLeftMotorInverted = false;
    inline constexpr bool kRightMotorInverted = true;

    inline constexpr double kMotorGearRatio = 9.0;

    inline constexpr units::meter_t kDrumRadius = 1_in;
    inline constexpr auto kCircumference =
        2.0 * std::numbers::pi_v<double> * kDrumRadius;

    inline constexpr units::kilogram_square_meter_t kMass = 1_kg_sq_m;

    inline constexpr int kNumMotors = 2;
    inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(kNumMotors);

    inline constexpr double kMotorEfficiency = 0.9;



    // inline constexpr units::meters_per_second_t kMaxVelocity =
    //     (kMotor.freeSpeed / kMotorGearRatio) * kUpperDrumRadius / 1_rad;

    //  inline constexpr units::radians_per_second_t kMaxVelocity = 3_rad_per_s;
    //  inline constexpr units::radians_per_second_squared_t kMaxAccel = 5_rad_per_s_sq;
    //  inline constexpr frc::TrapezoidProfile<units::radians_per_second>::Constraints kConstraints{kMaxVelocity, kMaxAccel};

    //  inline constexpr units::revolutions_per_minute_t kRPMTolerance = 4000_rpm;
     inline constexpr double kP = 20.0;
     inline constexpr double kI = 0.0;
     inline constexpr double kD = 0.0;
     inline constexpr units::millisecond_t kPeriod = 20_ms;
     
} 

class IntakeRoller : public LoggedSubsystem {
 public:
  explicit IntakeRoller(std::unique_ptr<IntakeRollerIO> io);
  ~IntakeRoller() = default;
 void SetIntakeRPM(units::radians_per_second_t desiredRPM);
  void SetVoltage(units::volt_t);

 protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

 private:
  std::unique_ptr<IntakeRollerIO> m_io;
  IntakeRollerIOInputs m_inputs{};
  units::meter_t m_desiredVelocity;
};