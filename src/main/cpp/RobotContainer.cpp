// Team 5687 2026

// RobotContainer.cpp
#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/voltage.h>

#include <array>
#include <memory>

#include "HardwareMap.h"
#include "commands/drive/DriveMaintainingHeadingCommand.h"
#include "commands/intake/EjectIntakeCommand.h"
#include "commands/intake/IntakeCommand.h"
#include "commands/shooter/ShootCommand.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "subsystem/drive/PigeonIO.h"
#include "subsystem/drive/SimGyroIO.h"
#include "subsystem/drive/module/CTREModuleIO.h"
#include "subsystem/drive/module/ModuleConfig.h"
#include "subsystem/drive/module/SimModuleIO.h"
#include "subsystem/floorroller/CTREFloorRollerIO.h"
#include "subsystem/floorroller/FloorRollerSubsystem.h"
#include "subsystem/floorroller/SimFloorRollerIO.h"
#include "subsystem/flywheel/CTREFlywheelIO.h"
#include "subsystem/flywheel/SimFlywheelIO.h"
#include "subsystem/hood/CTREHoodIO.h"
#include "subsystem/hood/SimHoodIO.h"
#include "subsystem/intake/IntakeSystem.h"
#include "subsystem/intake/bottomroller/CTREIntakeBottomRollerIO.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/bottomroller/SimIntakeBottomRollerIO.h"
#include "subsystem/intake/deployer/CTREIntakeDeployerIO.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/deployer/SimIntakeDeployerIO.h"
#include "subsystem/intake/toproller/CTREIntakeTopRollerIO.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"
#include "subsystem/intake/toproller/SimIntakeTopRollerIO.h"
#include "subsystem/kicker/CTREKickerIO.h"
#include "subsystem/kicker/KickerSubsystem.h"
#include "subsystem/kicker/SimKickerIO.h"
#include "subsystem/vision/LimelightVisionIO.h"
#include "subsystem/vision/SimVisionIO.h"

RobotContainer::RobotContainer() {
  m_drive = CreateDrive();
  m_flywheel = CreateFlywheel();
  m_hood = CreateHood();
  m_vision = CreateVision();
  m_floorRoller = CreateFloorRoller();
  m_kicker = CreateKicker();
  m_intakeDeployer = CreateIntakeDeployer();
  m_intakeTopRoller = CreateIntakeTopRoller();
  m_intakeBottomRoller = CreateIntakeBottomRoller();
  m_intake = std::make_unique<IntakeSystem>(
      *m_intakeDeployer, *m_intakeTopRoller, *m_intakeBottomRoller);
  m_shooter = std::make_unique<ShooterSystem>(*m_flywheel, *m_hood);
  ConfigureBindings();
}

std::unique_ptr<DriveSubsystem> RobotContainer::CreateDrive() {
  // Module encoder offsets (tune these per robot)
  constexpr std::array<units::turn_t, 4> kEncoderOffsets{
      0.405517578125_tr, // FL
      -0.2236328125_tr,  // FR
      0.33642578125_tr,  // BL
      -0.0517578125_tr   // BR
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

std::unique_ptr<FlywheelSubsystem> RobotContainer::CreateFlywheel() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<FlywheelSubsystem>(
        std::make_unique<SimFlywheelIO>());
  }
  return std::make_unique<FlywheelSubsystem>(std::make_unique<CTREFlywheelIO>(
      HardwareMap::CAN::TalonFX::LeftLeaderFlywheel,
      HardwareMap::CAN::TalonFX::LeftFollowerFlywheel,
      HardwareMap::CAN::TalonFX::RightFollowerFlywheel,
      HardwareMap::CAN::TalonFX::RightBottomFollowerFlywheel));
}

std::unique_ptr<FloorRollerSubsystem> RobotContainer::CreateFloorRoller() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<FloorRollerSubsystem>(
        std::make_unique<SimFloorRollerIO>());
  }
  return std::make_unique<FloorRollerSubsystem>(
      std::make_unique<CTREFloorRollerIO>(
          HardwareMap::CAN::TalonFX::FloorRollerLeader,
          HardwareMap::CAN::TalonFX::FloorRollerFollower));
}

std::unique_ptr<KickerSubsystem> RobotContainer::CreateKicker() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<KickerSubsystem>(std::make_unique<SimKickerIO>());
  }
  return std::make_unique<KickerSubsystem>(std::make_unique<CTREKickerIO>(
      HardwareMap::CAN::TalonFX::KickerLeader,
      HardwareMap::CAN::TalonFX::KickerFollower));
}

std::unique_ptr<IntakeDeployerSubsystem>
RobotContainer::CreateIntakeDeployer() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<IntakeDeployerSubsystem>(
        std::make_unique<SimIntakeDeployerIO>());
  }
  return std::make_unique<IntakeDeployerSubsystem>(
      std::make_unique<CTREIntakeDeployerIO>(
          HardwareMap::CAN::TalonFX::IntakeDeployer));
}

std::unique_ptr<IntakeTopRollerSubsystem>
RobotContainer::CreateIntakeTopRoller() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<IntakeTopRollerSubsystem>(
        std::make_unique<SimIntakeTopRollerIO>());
  }
  return std::make_unique<IntakeTopRollerSubsystem>(
      std::make_unique<CTREIntakeTopRollerIO>(
          HardwareMap::CAN::TalonFX::IntakeTopRollerLeader,
          HardwareMap::CAN::TalonFX::IntakeTopRollerFollower));
}

std::unique_ptr<IntakeBottomRollerSubsystem>
RobotContainer::CreateIntakeBottomRoller() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<IntakeBottomRollerSubsystem>(
        std::make_unique<SimIntakeBottomRollerIO>());
  }
  return std::make_unique<IntakeBottomRollerSubsystem>(
      std::make_unique<CTREIntakeBottomRollerIO>(
          HardwareMap::CAN::TalonFX::IntakeBottomRoller));
}

std::unique_ptr<HoodSubsystem> RobotContainer::CreateHood() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<HoodSubsystem>(std::make_unique<SimHoodIO>());
  }
  return std::make_unique<HoodSubsystem>(
      std::make_unique<CTREHoodIO>(HardwareMap::CAN::TalonFX::HoodMotor,
                                   HardwareMap::CAN::CANCoder::HoodEncoder));
}

std::unique_ptr<VisionSubsystem> RobotContainer::CreateVision() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<VisionSubsystem>(std::make_unique<SimVisionIO>(),
                                             m_drive->GetOdometryThread());
  }
  std::vector<std::string> limelights{"limelight"};
  return std::make_unique<VisionSubsystem>(
      std::make_unique<LimelightVisionIO>(
          limelights, LimelightVisionIO::MegaTagMode::kMegaTag1),
      m_drive->GetOdometryThread());
}
void RobotContainer::Periodic() {
  // m_robotViz.Update();
  m_robotViz.Update();
  // m_robotViz.FutureViz(1_s);
}

void RobotContainer::ConfigureBindings() {
  // Set default drive command
  using frc2::cmd::Run;

  m_drive->SetDefaultCommand(DriveMaintainingHeadingCommand(
      m_drive.get(), [this] { return -m_driver.GetLeftY(); },
      [this] { return -m_driver.GetLeftX(); },
      [this] { return -m_driver.GetRightX(); },
      true)); // slew limiter

  //m_driver.Circle().WhileTrue(
  //    Run([this] { m_flywheel->SetRPM(600_rpm); }, {m_flywheel.get()}));
  //m_driver.Triangle().WhileTrue(
  //    Run([this] { m_kicker->SetVelocity(60_tps); }, {m_kicker.get()}));
  m_driver.Cross().WhileTrue(
    Run([this] { m_hood->SetPosition(0_deg); }, {m_hood.get()}));

  m_driver.L2().WhileTrue(IntakeCommand(m_intakeDeployer.get(),
                                        m_intakeTopRoller.get(),
                                        m_intakeBottomRoller.get())
                              .ToPtr());

  m_driver.R2().WhileTrue(ShootCommand(
                              m_drive.get(), m_flywheel.get(), m_hood.get(),
                              m_intakeBottomRoller.get(), m_floorRoller.get(),
                              m_kicker.get(),
                              [this] { return -m_driver.GetLeftY(); },
                              [this] { return -m_driver.GetLeftX(); })
                             .ToPtr());

  m_driver.Square().WhileTrue(EjectIntakeCommand(m_intakeDeployer.get(),
                                        m_intakeTopRoller.get(),
                                        m_intakeBottomRoller.get())
                              .ToPtr());


    m_driver.R1().OnFalse(
        Run( [this] {
        m_flywheel->SetRPM(0_rpm);
        m_kicker->Stop();
        m_hood->SetPosition(0_deg);
        m_floorRoller->Stop();
      },
      {m_flywheel.get(), m_kicker.get(), m_floorRoller.get(), m_hood.get()}));

  m_driver.R1().WhileTrue(Run( [this] {
        m_flywheel->SetRPM(1000_rpm);
        m_kicker->SetVelocity(60_tps);
        m_hood->SetPosition(10_deg);
        if (m_flywheel->AtSetpoint()) {
          m_floorRoller->SetVoltage(10_V);
        } else {
          m_floorRoller->Stop();
        }
      },
      {m_flywheel.get(), m_kicker.get(), m_floorRoller.get(), m_hood.get()}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
