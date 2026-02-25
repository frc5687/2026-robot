// Team 5687 2026

#pragma once

#include <frc/geometry/Pose2d.h>

#include <vector>

#include "VisionIO.h"
#include "subsystem/vision/LimelightCamera.h"

class LimelightVisionIO : public VisionIO {
public:
  using MegaTagMode = LimelightCamera::MegaTagMode;

  explicit LimelightVisionIO(std::vector<LimelightCamera::Config> cameras);

  void SetRobotPose(const frc::Pose2d &pose) override;
  void UpdateInputs(VisionIOInputs &inputs) override;

private:
  std::vector<LimelightCamera> m_cameras;
};
