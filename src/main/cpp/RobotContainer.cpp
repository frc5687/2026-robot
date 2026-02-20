// Team 5687 2026

// RobotContainer.cpp
#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/length.h>

#include <array>
#include <memory>

#include "HardwareMap.h"
#include "commands/drive/DriveMaintainingHeadingCommand.h"
#include "subsystem/drive/PigeonIO.h"
#include "subsystem/drive/SimGyroIO.h"
#include "subsystem/drive/module/CTREModuleIO.h"
#include "subsystem/drive/module/ModuleConfig.h"
#include "subsystem/drive/module/SimModuleIO.h"
#include "subsystem/flywheel/CTREFlywheelIO.h"
#include "subsystem/flywheel/SimFlywheelIO.h"
#include "subsystem/hood/SimHoodIO.h"
#include "subsystem/indexer/CTREIndexerIO.h"
#include "subsystem/indexer/IndexerSubsystem.h"
#include "subsystem/indexer/SimIndexerIO.h"
#include "subsystem/intake/CTREIntakeIO.h"
#include "subsystem/intake/IntakeSubsystem.h"
#include "subsystem/intake/SimIntakeIO.h"
#include "subsystem/turret/CTRETurretIO.h"
#include "subsystem/turret/SimTurretIO.h"
#include "subsystem/turret/TurretSubsystem.h"
#include "subsystem/vision/LimelightVisionIO.h"
#include "subsystem/vision/SimVisionIO.h"

RobotContainer::RobotContainer() {
  m_drive = CreateDrive();
  m_turret = CreateTurret();
  m_flywheel = CreateFlywheel();
  m_hood = CreateHood();
  m_vision = CreateVision();
  m_indexer = CreateIndexer();
  m_intake = CreateIntake();
  m_shooter = std::make_unique<ShooterSystem>(*m_turret, *m_flywheel, *m_hood);
  ConfigureBindings();
}

std::unique_ptr<DriveSubsystem> RobotContainer::CreateDrive() {
  // Module encoder offsets (tune these per robot)
  constexpr std::array<units::turn_t, 4> kEncoderOffsets{
      0.3505859375_tr,           // FL
      -0.05517578125_tr,         // FR
      0.27099609375_tr - 0.5_tr, // BL
      0.096923828125_tr          // BR
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
          CTREModuleIO::DeviceIDs{HardwareMap::CAN::TalonFX::FrontLeftDrive,
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
          CTREModuleIO::DeviceIDs{HardwareMap::CAN::TalonFX::BackLeftDrive,
                                  HardwareMap::CAN::TalonFX::BackLeftSteer,
                                  HardwareMap::CAN::CANCoder::BackLeftEncoder},
          ModuleConfig{ModulePosition::BackLeft, kEncoderOffsets[2]}),

      std::make_unique<CTREModuleIO>(
          CTREModuleIO::DeviceIDs{HardwareMap::CAN::TalonFX::BackRightDrive,
                                  HardwareMap::CAN::TalonFX::BackRightSteer,
                                  HardwareMap::CAN::CANCoder::BackRightEncoder},
          ModuleConfig{ModulePosition::BackRight, kEncoderOffsets[3]}),

      std::make_unique<PigeonIO>(HardwareMap::CAN::Pidgeon2::IMU));
}

std::unique_ptr<TurretSubsystem> RobotContainer::CreateTurret() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<TurretSubsystem>(std::make_unique<SimTurretIO>());
  }
  return std::make_unique<TurretSubsystem>(std::make_unique<CTRETurretIO>(
      HardwareMap::CAN::TalonFX::Turret, HardwareMap::DIO::TurretHallEffect));
}

std::unique_ptr<FlywheelSubsystem> RobotContainer::CreateFlywheel() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<FlywheelSubsystem>(
        std::make_unique<SimFlywheelIO>());
  }
  return std::make_unique<FlywheelSubsystem>(std::make_unique<CTREFlywheelIO>(
      HardwareMap::CAN::TalonFX::LeftLeaderFlywheel,
      HardwareMap::CAN::TalonFX::LeftFollowerFlywheel,
      HardwareMap::CAN::TalonFX::RightLeaderFlywheel,
      HardwareMap::CAN::TalonFX::RightFollowerFlywheel));
}

std::unique_ptr<IndexerSubsystem> RobotContainer::CreateIndexer() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<IndexerSubsystem>(std::make_unique<SimIndexerIO>());
  }
  return std::make_unique<IndexerSubsystem>(std::make_unique<CTREIndexerIO>(
      HardwareMap::CAN::TalonFX::LeftIndexer,
      HardwareMap::CAN::TalonFX::RightIndexer,
      HardwareMap::CAN::TalonFX::CenterIndexer));
}

std::unique_ptr<IntakeSubsystem> RobotContainer::CreateIntake() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<IntakeSubsystem>(std::make_unique<SimIntakeIO>());
  }
  return std::make_unique<IntakeSubsystem>(std::make_unique<CTREIntakeIO>(
      HardwareMap::CAN::TalonFX::LeftRollerMotor,
      HardwareMap::CAN::TalonFX::RightRollerMotor));
}

std::unique_ptr<HoodSubsystem> RobotContainer::CreateHood() {
  return std::make_unique<HoodSubsystem>(std::make_unique<SimHoodIO>());
}

std::unique_ptr<VisionSubsystem> RobotContainer::CreateVision() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<VisionSubsystem>(std::make_unique<SimVisionIO>(),
                                             m_drive->GetOdometryThread());
  }
  std::vector<std::string> limelights{
      "limelight-br",
      "limelight-bl",
      "limelight-fr",
      "limelight-fl",
  };
  return std::make_unique<VisionSubsystem>(
      std::make_unique<LimelightVisionIO>(
          limelights, LimelightVisionIO::MegaTagMode::kMegaTag1),
      m_drive->GetOdometryThread());
}
void RobotContainer::Periodic() {
  // m_robotViz.Update();
  m_robotViz.Update();
  m_robotViz.FutureViz(1_s);
}

void RobotContainer::ConfigureBindings() {
  using frc2::cmd::Run;

  // Set default drive command
  m_drive->SetDefaultCommand(DriveMaintainingHeadingCommand(
      m_drive.get(), [this] { return -m_driver.GetLeftY(); },
      [this] { return -m_driver.GetLeftX(); },
      [this] { return -m_driver.GetRightX(); },
      true)); // slew limiter

  // m_driver.Square().WhileTrue(
  //     DriveWithNormalVectorAlignment(
  //         m_drive.get(),
  //         []() { return frc::Pose2d{5_m, 3_m, frc::Rotation2d{45_deg}}; },
  //         false)
  //         .ToPtr());
  // m_driver.Square().WhileTrue(
  //     Run([this] { m_turret->SetAngle(330_deg); }, {m_turret.get()}));

  // m_driver.Circle().WhileTrue(
  //     Run([this] { m_turret->SetAngle(180_deg); }, {m_turret.get()}));

  //  m_driver.Triangle().WhileTrue(
  //      Run([this] { m_turret->SetAngle(30_deg); }, {m_turret.get()}));
  //
  m_driver.L2().WhileTrue(
      Run([this] { m_intake->SetVoltage(10_V); }, {m_intake.get()}));

  m_driver.Circle().WhileTrue(
      Run([this] { m_indexer->SetVoltage(10_V); }, {m_indexer.get()}));
  m_driver.Square().WhileTrue(Run(
      [this] { m_flywheel->SetRPM(2000_rpm, 2000_rpm); }, {m_flywheel.get()}));
  // m_driver.Circle().WhileTrue(
  //     Run([this] { m_flywheel->SetRPM(0_rpm, 0_rpm); }, {m_flywheel.get()}));
  // m_driver.Triangle().WhileTrue(Run(
  //    [this] { m_flywheel->SetRPM(1000_rpm, 1000_rpm); },
  //    {m_flywheel.get()}));
  // m_driver.Cross().OnTrue(m_flywheel->SysIdDynamic(frc2::sysid::Direction::kForward));
  // m_driver.Circle().OnTrue(m_flywheel->SysIdDynamic(frc2::sysid::Direction::kReverse));
  // m_driver.Triangle().OnTrue(m_flywheel->SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // m_driver.Square().OnTrue(m_flywheel->SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  m_driver.Triangle().WhileTrue(
      Run([this] { m_shooter->SetState(ShooterState::TRACKING); }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
