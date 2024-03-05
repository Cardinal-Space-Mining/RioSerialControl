// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <array>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/SerialPort.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>

#include "SenderNT.h"
#include "../../../PiSerialControl/include/SerialInterface.h"


using namespace ctre::phoenix6;

#define BUF_SIZE 16

constexpr const char XON = 0x11;
constexpr const char XOFF = 0x13;

constexpr const char* RIO_CAN_BUS = "rio";

constexpr int PIGEON_CAN_ID = 0;  // << Value!!!
// motor ids...


class Robot : public frc::TimedRobot, wpi::Sendable {
public:
  Robot() = default;
  ~Robot() = default;

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

  void InitSendable(wpi::SendableBuilder&) override; // use for logging motor data

protected:
  void DisableAllMotors();

  void SerialPeriodic();

  void handle_motor_data_struct(const struct MotorDataStruct& mds);


private:
  frc::SerialPort serial = frc::SerialPort( 230400, frc::SerialPort::Port::kOnboard, 8, frc::SerialPort::Parity::kParity_None, frc::SerialPort::StopBits::kStopBits_One );
  bool serial_enable = false;
  std::array<hardware::TalonFX, 1> motors = { { hardware::TalonFX(1, RIO_CAN_BUS) } };  // << motor id constants!!!
  hardware::Pigeon2 pigeon_imu{ PIGEON_CAN_ID, RIO_CAN_BUS };

  SenderNT nt_sender{ "rio telemetry" };


};
