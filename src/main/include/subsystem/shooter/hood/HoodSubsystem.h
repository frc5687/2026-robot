#include "subsystem/LoggedSubsystem.h"
#include "HoodIO.h"


class HoodSubsystem : public LoggedSubsystem{

    public :
     explicit HoodSubsystem(std::unique_ptr<HoodIO> io);
     ~HoodSubsystem() = default;
     void SetHoodPosition();

    protected : 
    void UpdateInputs() override;
    void LogTelemetry() override;

    private :


};