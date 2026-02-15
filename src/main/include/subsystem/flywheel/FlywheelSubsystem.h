#pragma once

#include <memory>
#include "subsystem/LoggedSubsystem.h"
#include "FlywheelIO.h"
#include "subsystem/flywheel/FlywheelConstants.h"
#include "units/angular_velocity.h"
#include <frc/filter/LinearFilter.h>

class FlywheelSubsystem : public LoggedSubsystem {
public:
  explicit FlywheelSubsystem(std::unique_ptr<FlywheelIO> io);
  ~FlywheelSubsystem() = default;

  void SetRPM(units::revolutions_per_minute_t desiredRPMLeft, units::revolutions_per_minute_t desiredRPMRight);
protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<FlywheelIO> m_io;
  FlywheelIOInputs m_inputs{};

  units::revolutions_per_minute_t m_desiredRPMLeft{0_rpm};
  units::revolutions_per_minute_t m_desiredRPMRight{0_rpm};

  frc::LinearFilter<units::revolutions_per_minute_t> m_filter = frc::LinearFilter<units::revolutions_per_minute_t>::SinglePoleIIR(Constants::Flywheel::kFilterTime, Constants::Flywheel::kFilterPeriod);
};
