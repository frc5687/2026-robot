// Team 5687 2026

// DriveWithNormalVectorAlignment.h
#pragma once

#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <functional>
#include <optional>

#include "subsystem/drive/DriveSubsystem.h"

namespace Constants {
namespace DriveWithNormalVectorAlignment {
// Default constants - adjust these for your robot
constexpr units::meter_t kNormalVectorOffset = 0.5_m;
constexpr units::meter_t kBlendStart = 1.0_m;
constexpr units::meter_t kBlendStartAlgae = 0.8_m;
constexpr units::meter_t kBlendEnd = 0.1_m;

constexpr double kDriveKp = 3.0;
constexpr double kDriveKi = 0.0;
constexpr double kDriveKd = 0.0;
constexpr double kRotKp = 5.0;
constexpr double kRotKi = 0.0;
constexpr double kRotKd = 0.0;

constexpr units::meters_per_second_t kMaxVelocity = 3.0_mps;
constexpr units::meters_per_second_squared_t kMaxAcceleration = 2.0_mps_sq;
constexpr units::meter_t kPositionTolerance = 0.05_m;
constexpr units::meters_per_second_t kVelocityTolerance = 0.1_mps;
constexpr double kMinOutput = 0.1;
constexpr double kCounteractGain = 1.5;
constexpr double kAggressiveAccelMultiplier = 2.0;
constexpr double kSmoothingFactor = 0.5;
}  // namespace DriveWithNormalVectorAlignment
}  // namespace Constants

class DriveWithNormalVectorAlignment
    : public frc2::CommandHelper<frc2::Command,
                                 DriveWithNormalVectorAlignment> {
 public:
  DriveWithNormalVectorAlignment(DriveSubsystem* drive,
                                 std::function<frc::Pose2d()> finalPoseSupplier,
                                 bool isAlgae = false);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  void UpdateTargetPoses();
  frc::Pose2d GetCurrentTargetPose();
  void UpdatePIDGains();
  void UpdatePIDTolerances();

  // Subsystem
  DriveSubsystem* m_drive;

  // Pose suppliers
  std::function<frc::Pose2d()> m_finalPoseSupplier;

  // Target poses
  std::optional<frc::Pose2d> m_alignmentPose;
  std::optional<frc::Pose2d> m_finalPose;

  // PID Controllers
  frc::PIDController m_xController;
  frc::PIDController m_yController;
  frc::PIDController m_rotationController;

  // Velocity tracking
  frc::ChassisSpeeds m_fieldRelativeVelocity;

  // Configuration
  units::meter_t m_normalVectorOffset;
  units::meter_t m_blendStartDistance;
  units::meter_t m_blendEndDistance;
  units::meters_per_second_t m_maxVelocity;
  units::meters_per_second_squared_t m_maxAcceleration;
  units::meter_t m_positionTolerance;
  units::meters_per_second_t m_velocityTolerance;
  double m_minOutput;
  double m_counteractGain;
  double m_aggressiveAccelMultiplier;
  double m_smoothingFactor;

  bool m_isAlgae;
};
