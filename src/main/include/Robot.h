// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <array>
#include <list>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/SerialPort.h>
#include "frc/Joystick.h"
#include "frc/XboxController.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>

#include "ctre/Phoenix.h"

#include "SerialMgr.h"
#include "SenderNT.h"

// #include "SerialMgr.h"

using namespace ctre::phoenix6;

typedef ctre::phoenix6::hardware::TalonFX TalonFX6;
typedef WPI_TalonFX TalonFX5;

constexpr int PIGEON_CAN_ID = 0; // << Value!!!
// motor ids...

class Robot : public frc::TimedRobot, wpi::Sendable
{

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
  void DisableAllMotors();
  void DriveTrainControl();
  void HopperControl();
  void TrencherControl();
  void AutoDriveTrainControl();
  void AutoHopperControl();
  void AutoTrencherControl();
  void ConfigTracks();


  static void defaultVelocityCfg(TalonFX6& mtr);

  void InitSendable(wpi::SendableBuilder &) override; // use for logging motor data

private:
  SenderNT nt_sender{"rio telemetry"};

  std::array<ctre::phoenix6::hardware::TalonFX, 1> mts = {
    // TalonFX6(0) // track_right
    // TalonFX6(1), // track_left
    TalonFX6(0), // trencher
    // TalonFX6(3) // hopper_belt
    // TalonFX5(4)  // hopper_actuator
  };

  SerialMgr mgr{mts};

  // Motors. Note, Talon FX's Can only use the Pheonix 5 API, hence the F6 and F5 classes
  TalonFX6 track_right = 0;
  TalonFX6 track_left = 1;
  TalonFX6 trencher = 2;
  TalonFX6 hopper_belt = 3;
  TalonFX5 hopper_actuator = 4;

  // Joystick
  frc::Joystick logitech{0};

  double drive_power_scale_factor = .7;

  static constexpr auto TRENCHER_MAX_VELO = 50_tps;
  static constexpr auto HOPPER_BELT_MAX_VELO = 60_tps;
  static constexpr auto TRACKS_MAX_VELO = 125_tps;
};
