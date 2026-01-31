#include "subsystem/LoggedSubsystem.h"
#include "HoodIO.h"
#include "units/angle.h"


class HoodSubsystem : public LoggedSubsystem{

    public :
     explicit HoodSubsystem(std::unique_ptr<HoodIO> io);
     ~HoodSubsystem() = default;
     void SetHoodPosition(units::angle::turn_t);

    protected : 
    void UpdateInputs() override;
    void LogTelemetry() override;

    private :
        std::unique_ptr<HoodIO> m_io;
        HoodIOInputs m_inputs{};
        units::angle::turn_t hoodRotation;
};