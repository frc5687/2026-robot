#include "subsystem/LoggedSubsystem.h"
#include "HoodIO.h"
#include "units/angle.h"
#include "HoodState.h"

class HoodSubsystem : public LoggedSubsystem{

public:
    explicit HoodSubsystem(std::unique_ptr<HoodIO> io);
    ~HoodSubsystem() = default;
     void SetHoodPosition(units::angle::turn_t, units::angle::turn_t);
    HoodState GetHoodState() const;

protected :
    void UpdateInputs() override;
    void LogTelemetry() override;

private :
    std::unique_ptr<HoodIO> m_io;
    HoodIOInputs m_inputs{};
    units::angle::turn_t m_hoodRotation{0_tr};
    units::turn_t m_desiredAngle{0_tr};
};
