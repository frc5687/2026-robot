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
#include "subsystem/drive/PigeonIO.h"
#include "subsystem/drive/SimGyroIO.h"
#include "subsystem/drive/module/ModuleConfig.h"
#include "subsystem/drive/module/SimModuleIO.h"
#include "commands/drive/DriveWithNormalVectorAlignment.h"
#include "subsystem/drive/module/CTREModuleIO.h"

RobotContainer::RobotContainer() {
  m_drive = CreateDrive();
//   m_elevator = CreateElevator();
//   m_vision = CreateVision();
  ConfigureBindings();
}

std::unique_ptr<DriveSubsystem> RobotContainer::CreateDrive() {
  // Module encoder offsets (tune these per robot)
  constexpr std::array<units::turn_t, 4> kEncoderOffsets{
      0.079569_tr,               // FL
      0.43359375_tr - 0.5_tr,    // FR
      0.35595703125_tr - 0.5_tr, // BL
      -0.2431540625_tr + 0.5_tr  // BR
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

// std::unique_ptr<ElevatorSubsystem> RobotContainer::CreateElevator() {
//   if (frc::RobotBase::IsSimulation()) {
//     return std::make_unique<ElevatorSubsystem>(
//         std::make_unique<SimElevatorIO>());
//   }

//   return std::make_unique<ElevatorSubsystem>(
//       std::make_unique<CTREElevatorIO>(
//           HardwareMap::CAN::TalonFX::LeftElevator,
//           HardwareMap::CAN::TalonFX::RightElevator));
// }

// std::unique_ptr<VisionSubsystem> RobotContainer::CreateVision() {
//   return std::make_unique<VisionSubsystem>(
//       std::make_unique<SimVisionIO>(),
//       m_drive->GetOdometryThread());
// }

void RobotContainer::ConfigureBindings() {
  using frc2::cmd::Run;

  // Set default drive command
  m_drive->SetDefaultCommand(DriveMaintainingHeadingCommand(
      m_drive.get(),
      [this] { return -m_driver.GetLeftY(); },
      [this] { return -m_driver.GetLeftX(); },
      [this] { return -m_driver.GetRightX(); },
      false)); //s lew limiter

  m_driver.Square().WhileTrue(
      DriveWithNormalVectorAlignment(
          m_drive.get(),
          []() { return frc::Pose2d{5_m, 3_m, frc::Rotation2d{45_deg}}; },
          false)
      .ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}