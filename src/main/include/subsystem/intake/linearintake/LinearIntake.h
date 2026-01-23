#pragma once


#include "ctre/phoenix6/configs/MotionMagicConfigs.hpp"
#include "frc/simulation/DCMotorSim.h"
#include "frc/system/plant/DCMotor.h"
#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"
#include "units/moment_of_inertia.h"

#include <memory>
#include <units/length.h>

namespace Constants::LinearIntake {


    constexpr bool kLinearMotorInverted = false;

    constexpr double kP = 0.0;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;

    constexpr units::meter_t kCircumference = 0.0_m;
    constexpr double kGearRatio = 0.0;
    constexpr units::kilogram_square_meter_t kInertia = 1.0_kg_sq_m;

    constexpr units::meter_t maxExtension = 1.0_m;
    constexpr units::meter_t minExtension = 0.0_m;

    constexpr double kMaxVelocity = 1.0;
    constexpr double kMaxAccel = 2.0;

    constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60();

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