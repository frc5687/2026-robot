#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "subsystem/flywheel/FlywheelIO.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "utils/CANDevice.h"
#include <array>

using namespace ctre::phoenix6;

class CTREFlywheelIO : public FlywheelIO {
  public:
    CTREFlywheelIO(const CANDevice &motor);
    void UpdateInputs(FlywheelIOInputs& inputs) override;
    void SetFlywheelRPM(units::revolutions_per_minute_t desiredRPM) override;

  private:
    hardware::TalonFX m_motor;

    configs::TalonFXConfiguration m_config{};

    controls::VelocityVoltage m_request;

    StatusSignal<units::turns_per_second_t> &m_motorVelocitySignal;
    StatusSignal<units::ampere_t> &m_motorCurrentSignal;

    std::array<BaseStatusSignal*, 2> m_batchSignals;
};
