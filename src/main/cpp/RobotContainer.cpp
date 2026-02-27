// Team 5687 2026

// RobotContainer.cpp
#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/voltage.h>

#include <memory>

#include "HardwareMap.h"
#include "commands/drive/DriveMaintainingHeadingCommand.h"
#include "commands/intake/EjectIntakeCommand.h"
#include "commands/intake/IntakeCommand.h"
#include "commands/shooter/ShootCommand.h"
#include "subsystem/drive/PigeonIO.h"
#include "subsystem/drive/SimGyroIO.h"
#include "subsystem/drive/module/CTREModuleIO.h"
#include "subsystem/drive/module/ModuleConfig.h"
#include "subsystem/drive/module/SimModuleIO.h"
#include "subsystem/floorroller/CTREFloorRollerIO.h"
#include "subsystem/floorroller/SimFloorRollerIO.h"
#include "subsystem/flywheel/CTREFlywheelIO.h"
#include "subsystem/flywheel/SimFlywheelIO.h"
#include "subsystem/hood/CTREHoodIO.h"
#include "subsystem/hood/SimHoodIO.h"
#include "subsystem/intake/bottomroller/CTREIntakeBottomRollerIO.h"
#include "subsystem/intake/bottomroller/SimIntakeBottomRollerIO.h"
#include "subsystem/intake/deployer/CTREIntakeDeployerIO.h"
#include "subsystem/intake/deployer/SimIntakeDeployerIO.h"
#include "subsystem/intake/toproller/CTREIntakeTopRollerIO.h"
#include "subsystem/intake/toproller/SimIntakeTopRollerIO.h"
#include "subsystem/kicker/CTREKickerIO.h"
#include "subsystem/kicker/SimKickerIO.h"
#include "subsystem/vision/LimelightCamera.h"
#include "subsystem/vision/LimelightVisionIO.h"
#include "subsystem/vision/SimVisionIO.h"


constexpr std::array<units::turn_t, 4> kEncoderOffsets{
    +0.405517578125_tr, // FL
    +0.110595703125_tr, // FR
    +0.33642578125_tr,  // BL
    -0.0517578125_tr    // BR
};

std::unique_ptr<ModuleIO> MakeModuleIO(ModulePosition pos, units::turn_t offset,
                                       CTREModuleIO::DeviceIDs ids) {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimModuleIO>(ModuleConfig{pos, 0_tr});
  }
  return std::make_unique<CTREModuleIO>(ids, ModuleConfig{pos, offset});
}

std::unique_ptr<GyroIO> MakeGyroIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimGyroIO>();
  }
  return std::make_unique<PigeonIO>(HardwareMap::CAN::Pidgeon2::IMU);
}

std::unique_ptr<FlywheelIO> MakeFlywheelIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimFlywheelIO>();
  }
  return std::make_unique<CTREFlywheelIO>(
      HardwareMap::CAN::TalonFX::LeftLeaderFlywheel,
      HardwareMap::CAN::TalonFX::LeftFollowerFlywheel,
      HardwareMap::CAN::TalonFX::RightFollowerFlywheel,
      HardwareMap::CAN::TalonFX::RightBottomFollowerFlywheel);
}

std::unique_ptr<HoodIO> MakeHoodIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimHoodIO>();
  }
  return std::make_unique<CTREHoodIO>(HardwareMap::CAN::TalonFX::HoodMotor,
                                      HardwareMap::CAN::CANCoder::HoodEncoder);
}

std::unique_ptr<FloorRollerIO> MakeFloorRollerIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimFloorRollerIO>();
  }
  return std::make_unique<CTREFloorRollerIO>(
      HardwareMap::CAN::TalonFX::FloorRollerLeader,
      HardwareMap::CAN::TalonFX::FloorRollerFollower);
}

std::unique_ptr<KickerIO> MakeKickerIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimKickerIO>();
  }
  return std::make_unique<CTREKickerIO>(
      HardwareMap::CAN::TalonFX::KickerLeader,
      HardwareMap::CAN::TalonFX::KickerFollower);
}

std::unique_ptr<IntakeDeployerIO> MakeIntakeDeployerIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimIntakeDeployerIO>();
  }
  return std::make_unique<CTREIntakeDeployerIO>(
      HardwareMap::CAN::TalonFX::IntakeDeployer);
}

std::unique_ptr<IntakeTopRollerIO> MakeIntakeTopRollerIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimIntakeTopRollerIO>();
  }
  return std::make_unique<CTREIntakeTopRollerIO>(
      HardwareMap::CAN::TalonFX::IntakeTopRollerLeader,
      HardwareMap::CAN::TalonFX::IntakeTopRollerFollower);
}

std::unique_ptr<IntakeBottomRollerIO> MakeIntakeBottomRollerIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimIntakeBottomRollerIO>();
  }
  return std::make_unique<CTREIntakeBottomRollerIO>(
      HardwareMap::CAN::TalonFX::IntakeBottomRoller);
}

std::unique_ptr<VisionIO> MakeVisionIO() {
  if (frc::RobotBase::IsSimulation()) {
    return std::make_unique<SimVisionIO>();
  }
  std::vector<LimelightCamera::Config> limelights{
      {"limelight", LimelightCamera::MegaTagMode::kMegaTag1}};
  return std::make_unique<LimelightVisionIO>(std::move(limelights));
}

RobotContainer::RobotContainer()
    : m_drive{
          MakeModuleIO(ModulePosition::FrontLeft,  kEncoderOffsets[0],
                       {HardwareMap::CAN::TalonFX::FrontLeftDrive,
                        HardwareMap::CAN::TalonFX::FrontLeftSteer,
                        HardwareMap::CAN::CANCoder::FrontLeftEncoder}),
          MakeModuleIO(ModulePosition::FrontRight, kEncoderOffsets[1],
                       {HardwareMap::CAN::TalonFX::FrontRightDrive,
                        HardwareMap::CAN::TalonFX::FrontRightSteer,
                        HardwareMap::CAN::CANCoder::FrontRightEncoder}),
          MakeModuleIO(ModulePosition::BackLeft,   kEncoderOffsets[2],
                       {HardwareMap::CAN::TalonFX::BackLeftDrive,
                        HardwareMap::CAN::TalonFX::BackLeftSteer,
                        HardwareMap::CAN::CANCoder::BackLeftEncoder}),
          MakeModuleIO(ModulePosition::BackRight,  kEncoderOffsets[3],
                       {HardwareMap::CAN::TalonFX::BackRightDrive,
                        HardwareMap::CAN::TalonFX::BackRightSteer,
                        HardwareMap::CAN::CANCoder::BackRightEncoder}),
          MakeGyroIO()}
    , m_flywheel{MakeFlywheelIO()}
    , m_hood{MakeHoodIO()}
    , m_floorRoller{MakeFloorRollerIO()}
    , m_kicker{MakeKickerIO()}
    , m_intakeDeployer{MakeIntakeDeployerIO()}
    , m_intakeTopRoller{MakeIntakeTopRollerIO()}
    , m_intakeBottomRoller{MakeIntakeBottomRollerIO()}
    , m_vision{MakeVisionIO(), m_drive.GetOdometryThread()}
    , m_intake{m_intakeDeployer, m_intakeTopRoller, m_intakeBottomRoller}
    , m_shooter{m_flywheel, m_hood} {
  ConfigureBindings();
  ConfigureAutoCommands();
}

void RobotContainer::Periodic() { m_robotViz.Update(); }

void RobotContainer::ConfigureAutoCommands() {
  pathplanner::NamedCommands::registerCommand("Shoot", nullptr);
}

void RobotContainer::ConfigureBindings() {
  using frc2::cmd::Run;

  m_drive.SetDefaultCommand(DriveMaintainingHeadingCommand(
      &m_drive, [this] { return -m_driver.GetLeftY(); },
      [this] { return -m_driver.GetLeftX(); },
      [this] { return -m_driver.GetRightX(); },
      [this] { return m_driver.L1().Get(); },
      true));

  m_driver.Cross().WhileTrue(
      Run([this] { m_hood.SetPosition(0_deg); }, {&m_hood}));

  m_driver.L2().WhileTrue(IntakeCommand(&m_intakeDeployer, &m_intakeTopRoller,
                                        &m_intakeBottomRoller)
                              .ToPtr());

  m_driver.Options().WhileTrue(
      Run([this] { m_drive.ResetHeading(0_deg); }));

  m_driver.R2().WhileTrue(
      ShootCommand(&m_drive, &m_flywheel, &m_hood, &m_intakeBottomRoller,
                   &m_floorRoller, &m_kicker,
                   [this] { return -m_driver.GetLeftY(); },
                   [this] { return -m_driver.GetLeftX(); })
          .ToPtr());

  m_driver.Square().WhileTrue(
      EjectIntakeCommand(&m_intakeDeployer, &m_intakeTopRoller,
                         &m_intakeBottomRoller)
          .ToPtr());

  m_driver.R1().OnFalse(Run(
      [this] {
        m_flywheel.SetRPM(0_rpm);
        m_kicker.Stop();
        m_hood.SetPosition(0_deg);
        m_floorRoller.Stop();
        m_intakeBottomRoller.Stop();
      },
      {&m_flywheel, &m_kicker, &m_floorRoller, &m_hood,
       &m_intakeBottomRoller}));

  m_driver.R1().WhileTrue(Run(
      [this] {
        m_flywheel.SetRPM(1000_rpm);
        m_kicker.SetVelocity(60_tps);
        m_hood.SetPosition(10_deg);
        if (m_flywheel.AtSetpoint()) {
          m_floorRoller.SetVoltage(10_V);
          m_intakeDeployer.RetractMid();
          m_intakeBottomRoller.SetVoltage(10_V);
        } else {
          m_floorRoller.Stop();
        }
      },
      {&m_flywheel, &m_kicker, &m_floorRoller, &m_hood, &m_intakeBottomRoller,
       &m_intakeDeployer}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
