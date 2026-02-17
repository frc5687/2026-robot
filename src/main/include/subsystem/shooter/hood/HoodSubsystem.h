#include "subsystem/LoggedSubsystem.h"
#include "HoodIO.h"
#include "units/angle.h"


class HoodSubsystem : public LoggedSubsystem{

    public :
     explicit HoodSubsystem(std::unique_ptr<HoodIO> io);
     ~HoodSubsystem() = default;
     void SetHoodPosition(units::angle::turn_t leftHoodRotation, units::angle::turn_t rightHoodRotation);

    protected : 
    void UpdateInputs() override;
    void LogTelemetry() override;

    private :
        std::unique_ptr<HoodIO> m_io;
        HoodIOInputs m_inputs{};
        units::angle::turn_t m_leftDesiredSetpoint = 0.4_tr;
        units::angle::turn_t m_rightDesiredSetpoint = 0.4_tr;

        
};