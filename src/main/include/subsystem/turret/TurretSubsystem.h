#pragma once

#include <memory>
#include "subsystem/LoggedSubsystem.h"
#include "TurretIO.h"

class TurretSubsystem : public LoggedSubsystem {
public:
    explicit TurretSubsystem(std::unique_ptr<TurretIO> io);
    ~TurretSubsystem() = default;

    void SetAngle(units::radian_t desiredAngle);
protected:
    void UpdateInputs() override;
    void LogTelemetry() override;
private:
    std::unique_ptr<TurretIO> m_io;
    TurretIOInputs m_inputs{};

    units::radian_t m_desiredAngle{0_rad};
};
