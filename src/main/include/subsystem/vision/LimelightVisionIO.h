// Team 5687 2026

#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <string>
#include <vector>

#include "VisionIO.h"
#include "utils/vision/LimelightHelpers.h"

class LimelightVisionIO : public VisionIO {
public:
  enum class MegaTagMode {
    kMegaTag1,
    kMegaTag2,
  };

  explicit LimelightVisionIO(std::vector<std::string> limelightNames,
                             MegaTagMode mode = MegaTagMode::kMegaTag1);

  void SetRobotPose(const frc::Pose2d & /*pose*/) override;
  void
  SetRobotOrientation(units::degree_t yaw,
                      units::degrees_per_second_t yawRate = 0.0_deg_per_s,
                      units::degree_t pitch = 0.0_deg,
                      units::degrees_per_second_t pitchRate = 0.0_deg_per_s,
                      units::degree_t roll = 0.0_deg,
                      units::degrees_per_second_t rollRate = 0.0_deg_per_s);

  void UpdateInputs(VisionIOInputs &inputs) override;

private:
  void ProcessCamera(const std::string &name, VisionIOInputs &inputs);

  VisionMeasurement
  MeasurementFromEstimate(const std::string &limelightName,
                          const LimelightHelpers::PoseEstimate &est,
                          bool isMegaTag2) const;

  std::vector<std::string> m_limelightNames;
  MegaTagMode m_mode;
};
