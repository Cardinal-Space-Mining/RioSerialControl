// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/SerialPort.h>
#include "ctre/phoenix6/TalonFX.hpp"

using namespace ctre::phoenix6;

#define BUF_SIZE 16

constexpr const char XON = 0x11;
constexpr const char XOFF = 0x13;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

protected:
  void DisableAllMotors();

  void SerialPeriodic();

 private:
  frc::SerialPort serial = frc::SerialPort(230400,frc::SerialPort::Port::kOnboard, 8, frc::SerialPort::Parity::kParity_None, frc::SerialPort::StopBits::kStopBits_One );
  bool serial_enable = false;
  std::array<hardware::TalonFX,1> motors = {{hardware::TalonFX(1, "")}};
};