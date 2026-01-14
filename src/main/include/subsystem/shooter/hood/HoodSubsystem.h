#include "subsystem/LoggedSubsystem.h"
#include "CTREHoodIO.h"


class HoodSubsystem : public LoggedSubsystem{

    public :
     explicit HoodSubsystem(std::unique_ptr<HoodIO> io);
     ~HoodSubsystem() = default;
     void setHoodAngle();
    protected : 
    void UpdateInputs() override;
    void LogTelemetry() override;

    private :



};