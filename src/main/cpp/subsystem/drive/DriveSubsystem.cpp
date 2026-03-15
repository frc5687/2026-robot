// Team 5687 2026

#include "subsystem/drive/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "subsystem/vision/FieldConstants.h"
#include "subsystem/drive/SwerveDriveConstants.h"
#include "RobotState.h"
#include "frc/DriverStation.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "pathplanner/lib/config/ModuleConfig.h"
#include "pathplanner/lib/config/RobotConfig.h"
#include "subsystem/drive/PoseEstimator.h"
#include "units/velocity.h"

DriveSubsystem::DriveSubsystem(std::unique_ptr<ModuleIO> frontLeft,
                               std::unique_ptr<ModuleIO> frontRight,
                               std::unique_ptr<ModuleIO> backLeft,
                               std::unique_ptr<ModuleIO> backRight,
                               std::unique_ptr<GyroIO> gyro)
    : LoggedSubsystem("DriveSubsystem"),
      m_modules{std::make_unique<Module>(std::move(frontLeft)),
                std::make_unique<Module>(std::move(frontRight)),
                std::make_unique<Module>(std::move(backLeft)),
                std::make_unique<Module>(std::move(backRight))},
      m_gyro(std::move(gyro)), m_robotConfig(BuildRobotConfig()) {
  m_gyro->Reset();
  PoseEstimator::Config config{};
  config.odometryXStdDev = 0.08;       // Trust odometry more on smooth field
  config.baseXYStdDev = 0.15;          // Conservative vision trust
  config.odometryThetaStdDev = 0.0001; // Really trust IMU
  config.baseThetaStdDev = 0.2;        // Dont really trust camera rotations
  config.singleTagPenalty = 2.5;
  config.minConfidence = 0.1;
  m_odometryThread =
      std::make_unique<OdometryThread>(m_modules, m_gyro, config);
  StartOdometryThread();

  pathplanner::AutoBuilder::configure(
      [this]() { return GetPose(); },
      [this](frc::Pose2d pose) { ResetPose(pose); },
      [this]() { return GetChassisSpeeds(); },
      [this](auto speeds, auto feedforwards) { Drive(speeds); }, // relative
      std::make_shared<pathplanner::PPHolonomicDriveController>(
          pathplanner::PIDConstants(
              Constants::SwerveDrive::PID::Translation::kP,
              Constants::SwerveDrive::PID::Translation::kI,
              Constants::SwerveDrive::PID::Translation::kD),
          pathplanner::PIDConstants(Constants::SwerveDrive::PID::Rotation::kP,
                                    Constants::SwerveDrive::PID::Rotation::kI,
                                    Constants::SwerveDrive::PID::Rotation::kD)),
      m_robotConfig,
      []() {
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        return false;
      },
      this);
}

DriveSubsystem::~DriveSubsystem() { StopOdometryThread(); }

void DriveSubsystem::StartOdometryThread() {
  if (m_odometryThread) {
    m_odometryThread->Start();
    Log("OdometryThreadStarted", true);
  }
}

void DriveSubsystem::StopOdometryThread() {
  if (m_odometryThread) {
    m_odometryThread->Stop();
    Log("OdometryThreadStopped", true);
  }
}

void DriveSubsystem::SetOdometryFrequency(units::hertz_t frequency) {
  if (m_odometryThread) {
    m_odometryThread->SetUpdateFrequency(frequency);
  }
}

void DriveSubsystem::Drive(const frc::ChassisSpeeds &speeds) {
  frc::ChassisSpeeds discretizedSpeeds =
      frc::ChassisSpeeds::Discretize(speeds, 20_ms);
  std::array<frc::SwerveModuleState, 4> moduleStates =
      m_kinematics.ToSwerveModuleStates(discretizedSpeeds);
  SetModuleStates(moduleStates);
}

void DriveSubsystem::DriveFieldRelative(const frc::ChassisSpeeds &speeds) {
  std::optional<frc::DriverStation::Alliance> alliance =
      frc::DriverStation::GetAlliance();
  frc::Rotation2d relativeHeading = GetEstimatedHeading();

  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    relativeHeading = GetEstimatedHeading().RotateBy({180_deg});
  }
  auto robotRelative =
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, relativeHeading);
  Drive(robotRelative);
}

void DriveSubsystem::SetModuleStates(
    const std::array<frc::SwerveModuleState,
                     Constants::SwerveDrive::kModuleCount> &states) {
  wpi::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
      desaturatedStates = states;
  frc::SwerveDriveKinematics<Constants::SwerveDrive::kModuleCount>::
      DesaturateWheelSpeeds(
          &desaturatedStates,
          Constants::SwerveDrive::Module::kMaxModuleLinearSpeed);

  for (size_t i = 0; i < Constants::SwerveDrive::kModuleCount; i++) {
    m_modules[i]->SetDesiredState(desaturatedStates[i]);
    m_desiredStates[i] = desaturatedStates[i];
  }
}

void DriveSubsystem::Stop() {
  for (auto &module : m_modules) {
    module->Stop();
  }
}

void DriveSubsystem::LockWheels() {
  constexpr std::array<frc::SwerveModuleState,
                       Constants::SwerveDrive::kModuleCount>
      lockStates{frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}},
                 frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}},
                 frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}},
                 frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}}};
  SetModuleStates(lockStates);
}

std::pair<units::meters_per_second_t, units::radians_per_second_t>
DriveSubsystem::GetMaxSpeeds() const {
  return {m_maxLinearSpeed, m_maxAngularSpeed};
}

frc::Pose2d DriveSubsystem::GetPose() const {
  if (m_odometryThread) {
    return m_odometryThread->GetEstimatedPose();
  }
  return frc::Pose2d{};
}

frc::Rotation2d DriveSubsystem::GetHeading() const {
  if (m_odometryThread) {
    auto data = m_odometryThread->GetLatestData();
    return data.gyroAngle;
  }
  return frc::Rotation2d{};
}

frc::Rotation2d DriveSubsystem::GetEstimatedHeading() const {
  if (m_odometryThread) {
    return GetPose().Rotation();
  }
  return frc::Rotation2d{};
}

void DriveSubsystem::ResetHeading(units::degree_t heading) {
  m_gyro->Reset(heading);

  // Update pose with new heading
  auto currentPose = GetPose();
  ResetPose(frc::Pose2d{currentPose.Translation(), frc::Rotation2d{heading}});
}

frc::ChassisSpeeds DriveSubsystem::GetChassisSpeeds() const {
  if (m_odometryThread) {
    return m_odometryThread->GetChassisSpeeds();
  }
  return frc::ChassisSpeeds{};
}

frc::ChassisSpeeds DriveSubsystem::GetFieldRelativeSpeeds() const {
  auto robotSpeeds = GetChassisSpeeds();
  return frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotSpeeds,
                                                     GetEstimatedHeading());
}

std::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
DriveSubsystem::GetModuleStates() const {
  if (m_odometryThread) {
    return m_odometryThread->GetModuleStates();
  }

  // Fallback to direct module access
  return GetModuleData(&Module::GetState);
}

std::array<frc::SwerveModulePosition, Constants::SwerveDrive::kModuleCount>
DriveSubsystem::GetModulePositions() const {
  if (m_odometryThread) {
    return m_odometryThread->GetModulePositions();
  }

  // Fallback to direct module access
  return GetModuleData(&Module::GetPosition);
}

template <typename T>
std::array<T, Constants::SwerveDrive::kModuleCount>
DriveSubsystem::GetModuleData(T (Module::*getter)() const) const {
  std::array<T, Constants::SwerveDrive::kModuleCount> data;
  for (size_t i = 0; i < Constants::SwerveDrive::kModuleCount; i++) {
    data[i] = (m_modules[i].get()->*getter)();
  }
  return data;
}

// Statistics methods
units::second_t DriveSubsystem::GetOdometryLoopTime() const {
  if (m_odometryThread) {
    return m_odometryThread->GetAverageLoopTime();
  }
  return 0_s;
}

size_t DriveSubsystem::GetOdometrySuccessRate() const {
  if (m_odometryThread) {
    auto successful = m_odometryThread->GetSuccessfulBatches();
    auto failed = m_odometryThread->GetFailedBatches();
    if (successful + failed > 0) {
      return (successful * 100) / (successful + failed);
    }
  }
  return 0;
}
void DriveSubsystem::ResetPose(const frc::Pose2d &pose) {
  if (m_odometryThread) {
    m_odometryThread->ResetPose(pose);
  }
}

void DriveSubsystem::SetMaxSpeeds(units::meters_per_second_t linear,
                                  units::radians_per_second_t angular) {
  m_maxLinearSpeed = std::min(linear, Constants::SwerveDrive::kMaxLinearSpeed);
  m_maxAngularSpeed =
      std::min(angular, Constants::SwerveDrive::kMaxAngularSpeed);
}

void DriveSubsystem::SetBrakeMode(bool brake) {
  for (auto &module : m_modules) {
    module->SetBrakeMode(brake);
  }
}

void DriveSubsystem::ConfigureClosedLoop() {
  for (auto &module : m_modules) {
    module->ConfigureClosedLoop();
  }
}

std::array<bool, Constants::SwerveDrive::kModuleCount>
DriveSubsystem::GetModuleConnectionStatus() const {
  std::array<bool, Constants::SwerveDrive::kModuleCount> status;
  for (size_t i = 0; i < Constants::SwerveDrive::kModuleCount; i++) {
    status[i] = m_modules[i]->IsConnected();
  }
  return status;
}

bool DriveSubsystem::IsAtPose(const frc::Pose2d &pose,
                              units::meter_t tolerance) const {
  auto currentPose = GetPose();
  auto distance = currentPose.Translation().Distance(pose.Translation());
  return distance < tolerance;
}

void DriveSubsystem::SetCurrentLimits(units::ampere_t driveSupplyCurrentLimit,
                                     units::ampere_t steerSupplyCurrentLimit) {
  if (m_hasCurrentLimitConfig &&
      m_lastDriveSupplyCurrentLimit == driveSupplyCurrentLimit &&
      m_lastSteerSupplyCurrentLimit == steerSupplyCurrentLimit) {
    return;
  }

  for (auto &module : m_modules) {
    module->SetCurrentLimits(driveSupplyCurrentLimit, steerSupplyCurrentLimit);
  }
  m_lastDriveSupplyCurrentLimit = driveSupplyCurrentLimit;
  m_lastSteerSupplyCurrentLimit = steerSupplyCurrentLimit;
  m_hasCurrentLimitConfig = true;
}

void DriveSubsystem::SetAutoCurrentLimits() {
  SetCurrentLimits(Constants::SwerveDrive::Module::kDriveSupplyCurrentLimitAuto,
                   Constants::SwerveDrive::Module::kSteerSupplyCurrentLimitAuto);
}

void DriveSubsystem::SetTeleopCurrentLimits() {
  SetCurrentLimits(Constants::SwerveDrive::Module::kDriveSupplyCurrentLimitTeleop,
                   Constants::SwerveDrive::Module::kSteerSupplyCurrentLimitTeleop);
}

frc2::CommandPtr DriveSubsystem::GetPathCommand(frc::Pose2d currentPose){

  pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
    3.0_mps, 4.0_mps_sq,
    540_deg_per_s, 720_deg_per_s_sq);

  frc::Translation2d esttranslation = currentPose.Translation();
  double currentDegrees = currentPose.Rotation().Degrees().value();

  // Normalize to [-180, 180] and snap to nearest cardinal (0 or 180)
  double normalizedDeg = std::fmod(currentDegrees + 180.0, 360.0);
  if (normalizedDeg < 0) normalizedDeg += 360.0;
  normalizedDeg -= 180.0;
  frc::Rotation2d goalRotation = (std::abs(normalizedDeg) < 90.0)
    ? frc::Rotation2d(0_deg)
    : frc::Rotation2d(180_deg);

  frc::Pose2d goalPose = currentPose; // default: pathfind to self

  bool topHalf    = esttranslation.Y() > Constants::Field::kFieldWidth / 2.0;
  bool bottomHalf = !topHalf;
  double x = esttranslation.X().value();
  double L = Constants::Field::kFieldLength.value();

  if      (topHalf    && x > L * 0.25 && x < L * 0.50) goalPose = frc::Pose2d(Constants::Field::Trench::InsideTopBlue.Translation(),    goalRotation);
  else if (topHalf    &&                  x < L * 0.25) goalPose = frc::Pose2d(Constants::Field::Trench::OutsideTopBlue.Translation(),   goalRotation);
  else if (bottomHalf && x > L * 0.25 && x < L * 0.50) goalPose = frc::Pose2d(Constants::Field::Trench::InsideBottomBlue.Translation(), goalRotation);
  else if (bottomHalf &&                  x < L * 0.25) goalPose = frc::Pose2d(Constants::Field::Trench::OutsideBottomBlue.Translation(),goalRotation);
  else if (topHalf    && x > L * 0.50 && x < L * 0.75) goalPose = frc::Pose2d(Constants::Field::Trench::InsideTopRed.Translation(),     goalRotation);
  else if (topHalf    && x > L * 0.75)                  goalPose = frc::Pose2d(Constants::Field::Trench::OutsideTopRed.Translation(),    goalRotation);
  else if (bottomHalf && x > L * 0.50 && x < L * 0.75) goalPose = frc::Pose2d(Constants::Field::Trench::InsideBottomRed.Translation(),  goalRotation);
  else if (bottomHalf && x > L * 0.75)                  goalPose = frc::Pose2d(Constants::Field::Trench::OutsideBottomRed.Translation(), goalRotation);
  else goalPose = currentPose;
  return pathplanner::AutoBuilder::pathfindToPose(goalPose, constraints, 1.0_mps);
}


// This is handled by the odometry thread
void DriveSubsystem::UpdateInputs() {
  if (m_odometryThread) {
    m_cachedOdometry = m_odometryThread->GetLatestData(); // single lock
    RobotState::Instance().AddDriveObservation(m_cachedOdometry);
  }
}

void DriveSubsystem::LogTelemetry() {
  if (m_cachedOdometry.isValid) {
    Log("Odometry Pose", m_cachedOdometry.odometryPose);
    Log("Estimated Pose", m_cachedOdometry.estimatedPose);
    Log("ModuleStates", m_cachedOdometry.moduleStates);
    Log("Speeds", m_cachedOdometry.chassisSpeeds);
    auto fieldSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
        m_cachedOdometry.chassisSpeeds, m_cachedOdometry.gyroAngle);
    Log("FieldSpeeds", fieldSpeeds);
    Log("Gyro/Yaw", m_cachedOdometry.gyroAngle);
  }

  Log("DesiredModuleStates", m_desiredStates);
  Log("OdometryLoopTime", GetOdometryLoopTime().value() * 1000);
  Log("OdometrySuccessRate", static_cast<double>(GetOdometrySuccessRate()));

  auto connectionStatus = GetModuleConnectionStatus();
  bool allConnected =
      std::all_of(connectionStatus.begin(), connectionStatus.end(),
                  [](bool connected) { return connected; });
  Log("AllModulesConnected", allConnected);
}

pathplanner::RobotConfig DriveSubsystem::BuildRobotConfig() {
  pathplanner::ModuleConfig moduleConfig(
      Constants::SwerveDrive::Module::kWheelRadius,
      Constants::SwerveDrive::Module::kMaxModuleLinearSpeed,
      Constants::SwerveDrive::Module::kFrictionCoefficient,
      Constants::SwerveDrive::Module::kDriveMotor,
      Constants::SwerveDrive::Module::kDriveGearRatio,
      Constants::SwerveDrive::Module::kDriveSupplyCurrentLimitTeleop, 1.0);
  return pathplanner::RobotConfig(
      Constants::SwerveDrive::kMass, Constants::SwerveDrive::kMomentOfInertia,
      moduleConfig,
      std::vector<frc::Translation2d>(
          Constants::SwerveDrive::kModuleTranslations.begin(),
          Constants::SwerveDrive::kModuleTranslations.end()));
}
