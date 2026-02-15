#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp"
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
    CTREFlywheelIO(const CANDevice &rightLeaderMotor, const CANDevice &rightFollowerMotor, 
  const CANDevice &leftLeaderMotor, const CANDevice &leftFollowerMotor);
    void UpdateInputs(FlywheelIOInputs& inputs) override;
    void SetFlywheelRPM(units::revolutions_per_minute_t desiredRPMLeft, units::revolutions_per_minute_t desiredRPMRight) override;

  private:
    hardware::TalonFX m_rightLeaderMotor;
    hardware::TalonFX m_rightFollowerMotor;

    hardware::TalonFX m_leftLeaderMotor;
    hardware::TalonFX m_leftFollowerMotor;

    configs::TalonFXConfiguration m_rightLeaderConfig{};
    configs::TalonFXConfiguration m_leftLeaderConfig{};

    controls::VelocityTorqueCurrentFOC m_rightLeader;
    controls::Follower m_rightFollower;

    controls::VelocityTorqueCurrentFOC m_leftLeader;
    controls::Follower m_leftFollower;

    StatusSignal<units::turns_per_second_t> &m_rightLeaderVelocitySignal;
    StatusSignal<units::ampere_t> &m_rightLeaderCurrentSignal;

    StatusSignal<units::turns_per_second_t> &m_leftLeaderVelocitySignal;
    StatusSignal<units::ampere_t> &m_leftLeaderCurrentSignal;

    std::array<BaseStatusSignal*, 4> m_batchSignals;
};
