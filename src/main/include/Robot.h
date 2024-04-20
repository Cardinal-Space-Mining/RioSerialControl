// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/SerialPort.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/SerialPort.h>
#include "frc/Joystick.h"
#include "frc/XboxController.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "ctre/Phoenix.h"

using namespace ctre::phoenix6;

typedef ctre::phoenix6::hardware::TalonFX TalonFX6;
typedef WPI_TalonFX TalonFX5;

#define BUF_SIZE 13
#define MOTOR_COUNT 1

constexpr int PIGEON_CAN_ID = 0; // << Value!!!

class Robot : public frc::TimedRobot {
 public:
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
  void DisableAllMotors();
  void DriveTrainControl();
  void HopperControl();
  void TrencherControl();
  void ConfigTracks();

  static void defaultVelocityCfg(TalonFX6& mtr);

  // void InitSendable(wpi::SendableBuilder &) override; // use for loggin motor data

 private:
  // frc::SerialPort serial = frc::SerialPort(9600);
  frc::SerialPort serial = frc::SerialPort(9600, frc::SerialPort::Port::kOnboard, 8, frc::SerialPort::Parity::kParity_None, frc::SerialPort::StopBits::kStopBits_One);

  
  char * input_buffer = (char *)malloc(sizeof(char) * BUF_SIZE);
  uint16_t input_i = 0;
  char * xon = (char *)malloc(sizeof(char) * 1);
  char * xoff = (char *)malloc(sizeof(char) * 1);
  char * time_buffer = (char *)malloc(sizeof(char) * 13);
  bool serial_enable = false;

  // ctre::phoenix::motorcontrol::can::TalonFX motors[MOTOR_COUNT] = {ctre::phoenix::motorcontrol::can::TalonFX(1)};
  // std::array<ctre::pheonix6::hardware::TalonFX, 1> mts = {
  //   TalonFX6(0),
  // }
  ctre::phoenix6::hardware::TalonFX motors[MOTOR_COUNT] = {ctre::phoenix6::hardware::TalonFX(0)};

  // Consts for motors id's
  TalonFX6 track_right = 0;
  TalonFX6 track_left = 1;
  TalonFX6 trencher = 2;
  TalonFX6 hopper_belt = 3;
  TalonFX5 hopper_actuator = 4;


  frc::Joystick logitech{0};

  double drive_power_scale_factor = .7;

  static constexpr auto TRENCHER_MAX_VELO = 50_tps;
  static constexpr auto HOPPER_BELT_MAX_VELO = 60_tps;
  static constexpr auto TRACKS_MAX_VELO = 125_tps;
  
};
