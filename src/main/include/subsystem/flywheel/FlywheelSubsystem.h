#pragma once

#include <memory>
#include "subsystem/LoggedSubsystem.h"
#include "FlywheelIO.h"
#include "units/angular_velocity.h"
#include <frc/filter/LinearFilter.h>

class FlywheelSubsystem : public LoggedSubsystem {
public:
  explicit FlywheelSubsystem(std::unique_ptr<FlywheelIO> io);
  ~FlywheelSubsystem() = default;

  void SetRPM(units::revolutions_per_minute_t desiredRPM);
protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<FlywheelIO> m_io;
  FlywheelIOInputs m_inputs{};

  units::revolutions_per_minute_t m_desiredRPM{0_rpm};

  frc::LinearFilter<units::revolutions_per_minute_t> filter = frc::LinearFilter<units::revolutions_per_minute_t>::SinglePoleIIR(0.1, 0.02_s);  
};
