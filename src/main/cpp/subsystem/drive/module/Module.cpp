// Team 5687 2026

#include "subsystem/drive/module/Module.h"

#include <string>
#include <utility>

#include "Constants.h"
#include "subsystem/drive/SwerveDriveConstants.h"
#include "utils/Logger.h"

Module::Module(std::unique_ptr<ModuleIO> io)
    : m_io(std::move(io)), m_position(m_io->GetModuleConfig().modulePosition),
      m_name(ModulePositionToString(m_position)),
      m_logPrefix("Module/" + m_name + "/") {}

void Module::Periodic() {
  auto now = frc::Timer::GetFPGATimestamp();
  m_io->UpdateInputs(m_inputs, m_isSignalsBatched);
  if (now - m_lastLogTime >= Constants::kLogPeriod) {
    LogState();
    m_lastLogTime = now;
  }
}

void Module::SetDesiredState(const frc::SwerveModuleState &state) {
  SetDesiredState(state, true);
}

void Module::SetDesiredState(const frc::SwerveModuleState &state,
                             bool optimize) {
  m_desiredState = state;

  // if (optimize && ShouldOptimize(state)) {
  m_optimizedState = state;
  m_optimizedState.Optimize(m_inputs.steerAngle);
  m_optimizedState.CosineScale(m_inputs.steerAngle);
  //} else {
  //  m_optimizedState = state;
  //}

  m_io->SetDesiredState(m_optimizedState);
}

void Module::Stop() {
  m_io->Stop();
  m_desiredState = frc::SwerveModuleState{0_mps, m_inputs.steerAngle};
  m_optimizedState = m_desiredState;
}

frc::SwerveModuleState Module::GetState() const {
  return frc::SwerveModuleState{
      units::meters_per_second_t{
          m_velocityFilter.Calculate(m_inputs.driveVelocity.value())},
      m_inputs.steerAngle};
}

frc::SwerveModulePosition Module::GetPosition() const {
  return frc::SwerveModulePosition{m_inputs.drivePosition, m_inputs.steerAngle};
}

void Module::ResetDrivePosition() const { m_io->ResetDriveEncoder(); }

void Module::SetBrakeMode(bool brake) { m_io->SetBrakeMode(brake); }

void Module::ConfigureClosedLoop() { m_io->ConfigureClosedLoop(); }

void Module::SetCurrentLimits(units::ampere_t driveSupplyCurrentLimit,
                              units::ampere_t steerSupplyCurrentLimit) {
  m_io->SetCurrentLimits(driveSupplyCurrentLimit, steerSupplyCurrentLimit);
}

units::ampere_t Module::GetCurrentDraw() const {
  const auto totalCurrent = units::math::abs(m_inputs.driveCurrentDraw) +
                            units::math::abs(m_inputs.steerCurrentDraw);
  return units::ampere_t{m_currentFilter.Calculate(totalCurrent.value())};
}

bool Module::IsConnected() const {
  return m_inputs.driveConnected && m_inputs.steerConnected &&
         m_inputs.encoderConnected;
}

bool Module::ShouldOptimize(const frc::SwerveModuleState &desired) const {
  return units::math::abs(desired.speed) > kOptimizationThreshold;
}

void Module::LogState() {
  auto &logger = Logger::Instance();

  logger.Log(m_logPrefix + "Velocity", m_inputs.driveVelocity.value());
  logger.Log(m_logPrefix + "Position", m_inputs.drivePosition.value());
  logger.Log(m_logPrefix + "Angle", m_inputs.steerAngle);
  logger.Log(m_logPrefix + "SteerTurns", m_inputs.steerPosition.value());
  logger.Log(m_logPrefix + "CurrentState", GetState());
  logger.Log(m_logPrefix + "DesiredState", m_desiredState);
  logger.Log(m_logPrefix + "OptimizedState", m_optimizedState);
  logger.Log(m_logPrefix + "DriveCurrent", GetCurrentDraw().value());
  logger.Log(m_logPrefix + "DriveMotorCurrent",
             m_inputs.driveCurrentDraw.value());
  logger.Log(m_logPrefix + "SteerCurrent", m_inputs.steerCurrentDraw.value());
  logger.Log(m_logPrefix + "DriveTorque", GetDriveTorque().value());
  logger.Log(m_logPrefix + "DriveForce",
             (GetDriveTorque() / Constants::SwerveDrive::Module::kWheelRadius)
                 .value());
  logger.Log(m_logPrefix + "DriveVoltage", m_inputs.driveAppliedVolts.value());
  logger.Log(m_logPrefix + "SteerVoltage", m_inputs.steerAppliedVolts.value());
  logger.Log(m_logPrefix + "DriveTemp", m_inputs.driveTemperature.value());
  logger.Log(m_logPrefix + "SteerTemp", m_inputs.steerTemperature.value());
  logger.Log(m_logPrefix + "Connected", IsConnected());

  const auto velocityError = m_desiredState.speed - GetState().speed;
  const auto angleError =
      (m_optimizedState.angle - m_inputs.steerAngle).Radians();
  logger.Log(m_logPrefix + "VelocityError", velocityError.value());
  logger.Log(m_logPrefix + "AngleError", angleError.value());
}
