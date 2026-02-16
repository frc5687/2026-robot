#pragma once


#include "ctre/phoenix6/configs/MotionMagicConfigs.hpp"
#include "frc/simulation/DCMotorSim.h"
#include "frc/system/plant/DCMotor.h"
#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"
#include "units/acceleration.h"
#include "units/angle.h"
#include "units/mass.h"
#include "units/moment_of_inertia.h"
#include "units/velocity.h"

#include <memory>
#include <numbers>
#include <units/length.h>

namespace Constants::LinearIntake {


    inline constexpr bool kLinearMotorInverted = false;

    inline constexpr double kP = 5.0;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.05;
 
    inline constexpr double kGearRatio = 9.0;

    inline constexpr int kNumMotors = 2;
    inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(kNumMotors);

    inline constexpr units::meter_t kDrumRadius = 1.125_in;
    inline constexpr auto kCircumference =
    2.0 * std::numbers::pi_v<double> * kDrumRadius;

    inline constexpr units::mass::kilogram_t kMass = 1.0_kg;

    inline constexpr units::meter_t kMaxExtension = 1.0_m;
    inline constexpr units::meter_t kMinExtension = 0.0_m;

    inline constexpr units::meters_per_second_t kMaxVelocity =
    (kMotor.freeSpeed / kGearRatio) * kDrumRadius / 1_rad;
    inline constexpr units::meters_per_second_squared_t kMaxAccel = 2.0_mps_sq;

};

class LinearIntake : public LoggedSubsystem{
    public:
    explicit LinearIntake(std::unique_ptr<LinearIntakeIO> io);
    ~LinearIntake() = default;
    void SetPosition(units::meter_t);
    protected:
        void UpdateInputs() override;
        void LogTelemetry() override;

    private:
        std::unique_ptr<LinearIntakeIO> m_io;
        LinearIntakeIOInputs m_inputs{};
        units::meter_t m_desiredMeters;
};