// RobotContainer.cpp
#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/length.h>

#include <array>
#include <memory>
#include <utility>

#include "HardwareMap.h"
#include "commands/drive/DriveMaintainingHeadingCommand.h"
#include "rev/ServoChannel.h"
#include "rev/ServoHub.h"
#include "subsystem/drive/PigeonIO.h"
#include "subsystem/drive/SimGyroIO.h"
#include "subsystem/drive/module/ModuleConfig.h"
#include "subsystem/drive/module/SimModuleIO.h"
#include "commands/drive/DriveWithNormalVectorAlignment.h"
#include "subsystem/drive/module/CTREModuleIO.h"
#include "subsystem/flywheel/SimFlywheelIO.h"
#include "subsystem/flywheel/CTREFlywheelIO.h"  
#include "subsystem/shooter/hood/REVHoodIO.h"
#include "subsystem/vision/SimVisionIO.h"
#include "subsystem/shooter/hood/SimHoodIO.h"
#include "subsystem/shooter/hood/HoodIO.h"
#include "units/angular_velocity.h"
#include "utils/TunableDouble.h"

RobotContainer::RobotContainer():
    m_shooterRPM1("shooterrpm","rpm1", 1000),
    m_shooterRPM2("shooterrpm","rpm2", 1000)
 {
m_flywheel = CreateFlywheel();
  m_drive = CreateDrive();
  m_hood = CreateHood();
 // m_vision = CreateVision();
  ConfigureBindings();

}

std::unique_ptr<DriveSubsystem> RobotContainer::CreateDrive() {
  // Module encoder offsets (tune these per robot)
  constexpr std::array<units::turn_t, 4> kEncoderOffsets{
      0.3505859375_tr,               // FL
      -0.05517578125_tr,    // FR
      0.27099609375_tr - 0.5_tr, // BL
      0.096923828125_tr  // BR
      };

  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<DriveSubsystem>(
        std::make_unique<SimModuleIO>(
            ModuleConfig{ModulePosition::FrontLeft, 0_tr}),
        std::make_unique<SimModuleIO>(
            ModuleConfig{ModulePosition::FrontRight, 0_tr}),
        std::make_unique<SimModuleIO>(
            ModuleConfig{ModulePosition::BackLeft, 0_tr}),
        std::make_unique<SimModuleIO>(
            ModuleConfig{ModulePosition::BackRight, 0_tr}),
        std::make_unique<SimGyroIO>());
  }

  // Real hardware
  return std::make_unique<DriveSubsystem>(
      std::make_unique<CTREModuleIO>(
          CTREModuleIO::DeviceIDs{
              HardwareMap::CAN::TalonFX::FrontLeftDrive,
              HardwareMap::CAN::TalonFX::FrontLeftSteer,
              HardwareMap::CAN::CANCoder::FrontLeftEncoder},
          ModuleConfig{ModulePosition::FrontLeft, kEncoderOffsets[0]}),

      std::make_unique<CTREModuleIO>(
          CTREModuleIO::DeviceIDs{
              HardwareMap::CAN::TalonFX::FrontRightDrive,
              HardwareMap::CAN::TalonFX::FrontRightSteer,
              HardwareMap::CAN::CANCoder::FrontRightEncoder},
          ModuleConfig{ModulePosition::FrontRight, kEncoderOffsets[1]}),

      std::make_unique<CTREModuleIO>(
          CTREModuleIO::DeviceIDs{
              HardwareMap::CAN::TalonFX::BackLeftDrive,
              HardwareMap::CAN::TalonFX::BackLeftSteer,
              HardwareMap::CAN::CANCoder::BackLeftEncoder},
          ModuleConfig{ModulePosition::BackLeft, kEncoderOffsets[2]}),

      std::make_unique<CTREModuleIO>(
          CTREModuleIO::DeviceIDs{
              HardwareMap::CAN::TalonFX::BackRightDrive,
              HardwareMap::CAN::TalonFX::BackRightSteer,
              HardwareMap::CAN::CANCoder::BackRightEncoder},
          ModuleConfig{ModulePosition::BackRight, kEncoderOffsets[3]}),

      std::make_unique<PigeonIO>(HardwareMap::CAN::Pidgeon2::IMU));
}

std::unique_ptr<HoodSubsystem> RobotContainer::CreateHood(){
//    if (frc::RobotBase::IsSimulation()) {
//         return std::make_unique<HoodSubsystem>(std::make_unique<SimHoodIO>());
//     }
    return std::make_unique<HoodSubsystem>(
        std::make_unique<REVHoodIO>(
            1,
            HardwareMap::CAN::CANCoder::HoodEncoder
        )
    );
}

// std::unique_ptr<VisionSubsystem> RobotContainer::CreateVision() {
//   return std::make_unique<VisionSubsystem>(
//       std::make_unique<SimVisionIO>(),
//       m_drive->GetOdometryThread());
// }

std::unique_ptr<FlywheelSubsystem> RobotContainer::CreateFlywheel() {
    if (frc::RobotBase::IsSimulation()) {
        return std::make_unique<FlywheelSubsystem>(
            std::make_unique<SimFlywheelIO>());
    };

    return std::make_unique<FlywheelSubsystem>(
        std::make_unique<CTREFlywheelIO>(
            HardwareMap::CAN::TalonFX::RightFlywheel, HardwareMap::CAN::TalonFX::LeftFlywheel));
}

void RobotContainer::ConfigureBindings() {
  using frc2::cmd::Run;

  // Set default drive command
  m_drive->SetDefaultCommand(DriveMaintainingHeadingCommand(
      m_drive.get(),
      [this] { return -m_driver.GetLeftY(); },
      [this] { return -m_driver.GetLeftX(); },
      [this] { return -m_driver.GetRightX(); },
      false)); //s lew limiter

//   m_driver.Square().WhileTrue(
//       DriveWithNormalVectorAlignment(
//           m_drive.get(),
//           []() { return frc::Pose2d{5_m, 3_m, frc::Rotation2d{45_deg}}; },
//           false)
//       .ToPtr());

    m_driver.Circle().OnTrue(Run(
        [this] { m_flywheel->SetRPM(units::angular_velocity::revolutions_per_minute_t{m_shooterRPM1.Get()}); }, {m_flywheel.get()}));

    m_driver.Square().OnTrue(Run(
        [this] { m_flywheel->SetRPM(units::angular_velocity::revolutions_per_minute_t{3500});  }, {m_flywheel.get()}));
  
//    m_driver.Triangle().OnTrue(Run(
//       [this] { m_hood->SetHoodPosition(0.65_tr); }, {m_hood.get()})); 

//     m_driver.Square().OnTrue(Run(
//       [this] { m_hood->SetHoodPosition(0.25_tr); }, {m_hood.get()})); 

//     m_driver.Cross().OnTrue(Run(
//       [this] { m_hood->SetHoodPosition(0.2_tr); }, {m_hood.get()})); 

//      m_driver.Circle().OnTrue(Run(
//       [this] { m_hood->SetHoodPosition(0.0_tr); }, {m_hood.get()})); 
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
