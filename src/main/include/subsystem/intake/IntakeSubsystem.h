#pragma once


#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/IntakeRollerIO.h"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"
#include "units/voltage.h"
#include <memory>

class IntakeSubsystem : public LoggedSubsystem{
    public:
    explicit IntakeSubsystem(std::unique_ptr<IntakeRollerIO> io);
    ~IntakeSubsystem() = default;
    void SetVoltage(units::volt_t);
    protected:
        void UpdateInputs() override;
        void LogTelemetry() override;

    private:

        std::unique_ptr<IntakeRollerIO> m_io;
        IntakeRollerIOInputs m_inputs{};
        units::meter_t m_desiredMeters;
};