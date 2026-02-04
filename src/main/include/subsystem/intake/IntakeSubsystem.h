#pragma once


#include "subsystem/LoggedSubsystem.h"
#include "subsystem/intake/IntakeRollerIO.h"
#include "subsystem/intake/linearintake/LinearIntakeIO.h"
#include "units/voltage.h"
#include <memory>

class IntakeSubsystem : public LoggedSubsystem{
    public:
    explicit IntakeSubsystem(std::unique_ptr<LinearIntakeIO> lio, std::unique_ptr<IntakeRollerIO> rio);
    ~IntakeSubsystem() = default;
    void SetPosition(units::meter_t);
    void SetVoltage(units::volt_t);
    protected:
        void UpdateInputs() override;
        void LogTelemetry() override;

    private:

        std::unique_ptr<LinearIntakeIO> m_Lio;
        std::unique_ptr<IntakeRollerIO> m_Rio;
        LinearIntakeIOInputs m_Linputs{};
        IntakeRollerIOInputs m_Rinputs{};
        units::meter_t m_desiredMeters;
};