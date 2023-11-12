// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/SerialPort.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

#include "SenderNT.h"

#define BUF_SIZE 13
#define MOTOR_COUNT 1
#define LOG_LOCAL_TELEMETRY true
#define MC_SENDABLE_TELEMETRY true
#define SENDABLE_TELEMETRY_RATE 100_ms

class Robot : public frc::TimedRobot, wpi::Sendable {
 public:
  Robot();

  char* itoa(int i, char b[]);

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

  void InitSendable(wpi::SendableBuilder&) override;

 private:
  frc::SerialPort serial = frc::SerialPort(230400);
  char * input_buffer = (char *)malloc(sizeof(char) * BUF_SIZE);
  uint16_t input_i = 0;
  char * xon = (char *)malloc(sizeof(char) * 1);          // might wanna const and default initialize these
  char * xoff = (char *)malloc(sizeof(char) * 1);
  char * time_buffer = (char *)malloc(sizeof(char) * 13);
  bool serial_enable = false;

  ctre::phoenix::motorcontrol::can::TalonFX motors[MOTOR_COUNT] = { ctre::phoenix::motorcontrol::can::TalonFX(1) };
#if(MC_SENDABLE_TELEMETRY)
  std::vector<float> sendable_buffers[8] = {};
  SenderNT sender{ "Robot Telemetry" };
#endif

};
