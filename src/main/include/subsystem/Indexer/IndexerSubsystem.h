#pragma once

#include <memory>
#include "subsystem/LoggedSubsystem.h"
#include "IndexerIO.h"
#include "units/voltage.h"

class IndexerSubsystem : public LoggedSubsystem {
public:
    explicit IndexerSubsystem(std::unique_ptr<IndexerIO> io);
    ~IndexerSubsystem() = default;

    // void SetAngle(units::radian_t desiredAngle);
    void SetVoltage(units::volt_t voltage);
protected:
    void UpdateInputs() override;
    void LogTelemetry() override;
private:
    std::unique_ptr<IndexerIO> m_io;
    IndexerIOInputs m_inputs{};
    
    // units::radian_t m_desiredAngle{0_rad};
};