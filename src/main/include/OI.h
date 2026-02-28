// Team 5687 2026

#pragma once

#include "frc2/command/button/CommandPS5Controller.h"
#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"

class OI {
public:
  void ConfigureButtions(DriveSubsystem &drive, FlywheelSubsystem &flywheel,
                         HoodSubsystem &hood);

private:
  frc2::CommandPS5Controller m_driver{0};
};
