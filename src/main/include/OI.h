#pragma once

#include <frc2/command/button/CommandPS5Controller.h>

#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/indexer/IndexerSubsystem.h"
#include "subsystem/intake/IntakeSubsystem.h"
#include "subsystem/shooter/ShooterSystem.h"
#include "utils/TunableDouble.h"

class OI {
 public:
  OI(DriveSubsystem& drive, FlywheelSubsystem& flywheel,
     IndexerSubsystem& indexer, IntakeSubsystem& intake,
     ShooterSystem& shooter, HoodSubsystem& hood);

 private:
  frc2::CommandPS5Controller m_driver{0};

  void ConfigureDriverBindings(DriveSubsystem& drive,
                                FlywheelSubsystem& flywheel,
                                IndexerSubsystem& indexer,
                                IntakeSubsystem& intake,
                                ShooterSystem& shooter,
                                HoodSubsystem& hood);

    TunableDouble m_microseconds;
    TunableDouble m_hoodangle;
};