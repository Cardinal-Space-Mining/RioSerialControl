// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>
#include <string>
#include <deque>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/SerialPort.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/AnalogPotentiometer.h>
#include "frc/Joystick.h"
#include "frc/XboxController.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "ctre/Phoenix.h"

using namespace ctre::phoenix6;

typedef ctre::phoenix6::hardware::TalonFX TalonFX6;
typedef WPI_TalonFX TalonFX5;

#define BUF_SIZE 13
#define MOTOR_COUNT 5

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

  uint8_t StartMining();
  uint8_t StopMining();

  uint8_t StartOffload();
  uint8_t StopOffload();

  uint8_t getMovingAvg();

  void DriveTrainControl();
  void HopperControl();
  void TrencherControl();

  void TeleopControl();

  void ConfigTracks();

  static void defaultVelocityCfg(TalonFX6& mtr);

  // void InitSendable(wpi::SendableBuilder &) override; // use for loggin motor data

 private:
  // frc::SerialPort serial = frc::SerialPort(9600);
  frc::SerialPort serial = frc::SerialPort(115200, frc::SerialPort::Port::kOnboard, 8, frc::SerialPort::Parity::kParity_None, frc::SerialPort::StopBits::kStopBits_One);

  
  char * input_buffer = (char *)malloc(sizeof(char) * BUF_SIZE);
  uint16_t input_i = 0;
  char * xon = (char *)malloc(sizeof(char) * 1);
  char * xoff = (char *)malloc(sizeof(char) * 1);
  char * time_buffer = (char *)malloc(sizeof(char) * 13);
  bool serial_enable = false;

  bool is_mining = false;
  bool is_offload = false;
  bool finished_cycle = false;

  // ctre::phoenix::motorcontrol::can::TalonFX motors[MOTOR_COUNT] = {ctre::phoenix::motorcontrol::can::TalonFX(1)};
  // std::array<ctre::pheonix6::hardware::TalonFX, 1> mts = {
  //   TalonFX6(0),
  // }
  // Consts for motors id's
  TalonFX6 track_right = 0;
  TalonFX6 track_left = 1;
  TalonFX6 trencher = 2;
  TalonFX6 hopper_belt = 3;
  TalonFX5 hopper_actuator = 4;

  ctre::phoenix6::hardware::TalonFX* motors[2] = {
    &track_right,
    &track_left,
  };

  // Actuator Potentiometer
  double offload_depth = .95;
  double reg_traversal_depth = .5;
  double mining_to_offload_depth = .45;
  double mining_depth = .03;
  // frc::AnalogInput hopper_actuator_pot_input {0};
  frc::AnalogPotentiometer hopper_actuator_pot {0};

  frc::Joystick logitech{0};

  double drive_power_scale_factor = .7;

  static constexpr auto TRENCHER_MAX_VELO = 40_tps;
  static constexpr auto HOPPER_BELT_MAX_VELO = 40_tps;
  static constexpr auto HOPPER_BELT_MAX_MINING_VELO = 10_tps;
  static constexpr auto TRACKS_MAX_VELO = 125_tps;
  static constexpr auto TRACKS_MINING_MAX_VELO = 8_tps;
  static constexpr auto HOPPER_ACTUATOR_MAX_VELO = .2;  

  bool on_off = false;

  // timer has been started for run
  bool time_set = false;
  std::chrono::system_clock::time_point start_time;
  // constant for how long each mining run is in time (seconds)
  static constexpr auto mining_run_time = 10;
  // constant for how long each offload run is in time (seconds)
  bool is_offload_pos = false;
  std::chrono::system_clock::time_point start_move_time_off;
  static constexpr auto offload_move_time_teleop = 3;
  static constexpr auto offload_move_time_automatic = 1;
  static constexpr auto offload_run_time = 10;

  // constants for timing of hopper movement during mining
  static constexpr int hopper_belt_mine_wait_time = 750; // in milliseconds
  static constexpr int hopper_belt_mine_run_time = 100; // in milliseconds

  // Variables for moving average
  uint8_t movingAvgRange = 10;
  double trenchAvgCurrent = 0.0;
  const double safteyAvgCurrent = 50.0;
  std::deque<double> motorDataList;
};
