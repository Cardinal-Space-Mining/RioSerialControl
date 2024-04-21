// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <chrono>
#include <iostream>
// #include <sys/time.h>
#include <ctime>
#include <cmath>
#include <iostream>
#include "LogitechConstants.hpp"
using std::cout;
using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

void Robot::DisableAllMotors() {
  track_right.Set(0);
  track_left.Set(0);
  trencher.Set(0);
  hopper_belt.Set(0);
  hopper_actuator.Set(0);
}

void Robot::defaultVelocityCfg(TalonFX6 &motor) {
  configs::TalonFXConfiguration configs{};
  /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
  configs.Slot0.kP = 0.11;   // An error of 1 rotation per second results in 2V output
  configs.Slot0.kI = 0.5;    // An error of 1 rotation per second increases output by 0.5V every second
  configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.0001 volts output
  configs.Slot0.kV = 0.12;   // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
  motor.GetConfigurator().Apply(configs);
}

void Robot::ConfigTracks() {
  configs::TalonFXConfiguration configs{};
  /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
  configs.Slot0.kP = 0.11;   // An error of 1 rotation per second results in 2V output
  configs.Slot0.kI = 0.5;    // An error of 1 rotation per second increases output by 0.5V every second
  configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.0001 volts output
  configs.Slot0.kV = 0.12;   // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
  configs.CurrentLimits.StatorCurrentLimitEnable = true;
  configs.CurrentLimits.StatorCurrentLimit = 40; // Set max current in stator to 40A
  track_left.GetConfigurator().Apply(configs);
  track_right.GetConfigurator().Apply(configs);
}

/**
 * Define constants
 * xon & xoff start end commands for RIO
 */
void Robot::RobotInit() {
  // add sendables to sender, register update callback
  // this->nt_sender.putData(&this->pigeon_imu, "pigeon"); // need to test with actual device -- in sim this didn't add anything useful to NT
  this->nt_sender.putData(this, "robot");
  this->AddPeriodic([this] { this->nt_sender.updateValues(); }, 20_ms);
  DisableAllMotors();
  // runs every 1 millisec, this is what is actually reading input
  AddPeriodic([this] { this->mgr.serial_periodic(); }, 1_ms);
  Robot::defaultVelocityCfg(trencher);
  Robot::defaultVelocityCfg(hopper_belt);
  ConfigTracks();
  track_right.SetInverted(true);
}

/**
 * eventually maybe not do this
 */
void Robot::RobotPeriodic() {

}

/**
 * Zeroing out all motors and enables serial to listen for opcodes
 */
void Robot::AutonomousInit() {
  // serial.Reset();
  DisableAllMotors();
  mgr.enable();
}

/**
 * does nothing for now
 */
void Robot::AutonomousPeriodic() {
  mgr.enable();
}

/**
 * dont listen to serial, manual control of rover
 */
void Robot::TeleopInit() {
  mgr.disable();
}

static constexpr long double PI = 3.14159265358979323846;

void Robot::TrencherControl() {
//   double belt_percentage = -logitech.GetRawAxis(LogitechConstants::RIGHT_TRIGGER);
//   if (logitech.GetRawButton(LogitechConstants::RB))
//   {
//     belt_percentage = -1.0 * belt_percentage;
//   }

//   ctre::phoenix6::controls::VelocityVoltage trencherVelocity{TRENCHER_MAX_VELO * belt_percentage, 5_tr_per_s_sq, false, 0_V, 0, false};
//   trencher.SetControl(trencherVelocity);
}

void Robot::AutoTrencherControl() {
  bool mineForward = true; // true if collecting, false if backwards
  bool miningMode = true; // true if mining autonomously, false if autonomy is off or driving
  double belt_percentage = 0.0; // I believe percentage
  // TODO check for jam/moving average

  if (miningMode) {
    //TODO turn off tracks
    belt_percentage = 0.2;
    // TODO lower trencher
    // TODO move tracks forward for x seconds
    // TODO stop track movement if x seconds
    //        raise trencher
    //        stop trencher
    //        mining mode false
    if (!mineForward) {
      // in case trencher stuck will go backwards and forward
      belt_percentage *= -1.0;
    }
    ctre::phoenix6::controls::VelocityVoltage trencherVelocity{TRENCHER_MAX_VELO * belt_percentage, 5_tr_per_s_sq, false, 0_V, 0, false};
    trencher.SetControl(trencherVelocity);
  }
  else {
    //stop mining forward
    belt_percentage = 0;
    ctre::phoenix6::controls::VelocityVoltage trencherVelocity{TRENCHER_MAX_VELO * belt_percentage, 5_tr_per_s_sq, false, 0_V, 0, false};
    trencher.SetControl(trencherVelocity);
    // TODO lift trencher
    
  }
}

void Robot::HopperControl() {
//   // Control Hopper Belt
//   double belt_percentage = -logitech.GetRawAxis(LogitechConstants::LEFT_TRIGGER);
//   if (logitech.GetRawButton(LogitechConstants::LB))
//   {
//     belt_percentage = -1.0 * belt_percentage;
//   }

//   ctre::phoenix6::controls::VelocityVoltage belt_velo{HOPPER_BELT_MAX_VELO * belt_percentage, 5_tr_per_s_sq, false, 0_V, 0, false};
//   hopper_belt.SetControl(belt_velo);

//   // Control Hopper Actuator
//   double actuator_power = std::max(-logitech.GetRawAxis(LogitechConstants::RIGHT_JOY_Y), -0.1);
//   hopper_actuator.Set(-actuator_power);
}


void Robot::AutoHopperControl() {
  // Control Hopper Belt
  double belt_percentage = -logitech.GetRawAxis(LogitechConstants::LEFT_TRIGGER);
  if (logitech.GetRawButton(LogitechConstants::LB))
  {
    belt_percentage = -1.0 * belt_percentage;
  }

  ctre::phoenix6::controls::VelocityVoltage belt_velo{HOPPER_BELT_MAX_VELO * belt_percentage, 5_tr_per_s_sq, false, 0_V, 0, false};
  hopper_belt.SetControl(belt_velo);

  // Control Hopper Actuator
  double actuator_power = std::max(-logitech.GetRawAxis(LogitechConstants::RIGHT_JOY_Y), -0.1);
  hopper_actuator.Set(-actuator_power);
}


void Robot::DriveTrainControl() {
//   using namespace LogitechConstants;

//   ctre::phoenix6::controls::VelocityVoltage m_voltageVelocity{0_tps, 1_tr_per_s_sq, false, 0_V, 0, false};

//   const double stick_x = logitech.GetRawAxis(LEFT_JOY_X); 
//   const double stick_y = -logitech.GetRawAxis(LEFT_JOY_Y);

//   const double theta = std::atan2(stick_x, stick_y);
//   double magnitude = std::sqrt(std::pow(stick_x, 2) + std::pow(stick_y, 2));
//   if (magnitude < 0.1) {
//     magnitude = 0;
//   }

//   if (logitech.GetRawButton(LogitechConstants::BUTTON_X)) {
//     drive_power_scale_factor = 1;
//   }

//   if (logitech.GetRawButton(LogitechConstants::BUTTON_Y)) {
//     drive_power_scale_factor = 0.7;
//   }

//   if (logitech.GetRawButton(LogitechConstants::BUTTON_B)) {
//     drive_power_scale_factor = 0.3;
//   }

//   auto r_velo = drive_power_scale_factor * TRACKS_MAX_VELO * magnitude * std::cos(theta + (PI / 4));
//   auto l_velo = drive_power_scale_factor * TRACKS_MAX_VELO * magnitude * std::cos(theta - (PI / 4));

//   track_right.SetControl(m_voltageVelocity.WithVelocity(r_velo));
//   track_left.SetControl(m_voltageVelocity.WithVelocity(l_velo));
}


void Robot::AutoDriveTrainControl() {
  using namespace LogitechConstants;

  ctre::phoenix6::controls::VelocityVoltage m_voltageVelocity{0_tps, 1_tr_per_s_sq, false, 0_V, 0, false};

  const double stick_x = logitech.GetRawAxis(LEFT_JOY_X); 
  const double stick_y = -logitech.GetRawAxis(LEFT_JOY_Y);

  const double theta = std::atan2(stick_x, stick_y);
  double magnitude = std::sqrt(std::pow(stick_x, 2) + std::pow(stick_y, 2));
  if (magnitude < 0.1) {
    magnitude = 0;
  }

  if (logitech.GetRawButton(LogitechConstants::BUTTON_X)) {
    drive_power_scale_factor = 1;
  }

  if (logitech.GetRawButton(LogitechConstants::BUTTON_Y)) {
    drive_power_scale_factor = 0.7;
  }

  if (logitech.GetRawButton(LogitechConstants::BUTTON_B)) {
    drive_power_scale_factor = 0.3;
  }

  auto r_velo = drive_power_scale_factor * TRACKS_MAX_VELO * magnitude * std::cos(theta + (PI / 4));
  auto l_velo = drive_power_scale_factor * TRACKS_MAX_VELO * magnitude * std::cos(theta - (PI / 4));

  track_right.SetControl(m_voltageVelocity.WithVelocity(r_velo));
  track_left.SetControl(m_voltageVelocity.WithVelocity(l_velo));
}


/**
 * nothing, will eventuall put here
 */
void Robot::TeleopPeriodic() {
  this->DriveTrainControl();
  this->HopperControl();
  this->TrencherControl();
}

/**
 * Turns off all motors when disabled
 */
void Robot::DisabledInit() {
  // mgr.disable();
  DisableAllMotors();
}

/**
 * nothing until next major block
 */
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}
/**
 * end of nothing block
 */

void Robot::InitSendable(wpi::SendableBuilder &builder) {
  // builder.AddDoubleArrayProperty(
  //     "pigeon rotation quat", [this]
  //     {
  //   frc::Rotation3d r = this->pigeon_imu.GetRotation3d();
  //   static std::vector<double> _data;
  //   _data.resize(4);
  //   memcpy(_data.data(), &r.GetQuaternion(), sizeof(frc::Quaternion));
  //   return _data; },
  //     nullptr);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

