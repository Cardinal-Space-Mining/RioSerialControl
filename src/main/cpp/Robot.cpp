// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

#include "../../../PiSerialControl/include/SerialInterface.h"

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>

using std::cout;
using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

namespace
{
  configs::TalonFXConfiguration get_default_motor_cfg()
  {
    configs::TalonFXConfiguration configs{};

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11;   // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5;    // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.0001 volts output
    configs.Slot0.kV = 0.12;   // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

    configs.Voltage.PeakForwardVoltage = 8;  // Peak output of 8 volts
    configs.Voltage.PeakReverseVoltage = -8; // Peak output of 8 volts

    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kP = 5;     // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = 0.1;   // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;  // Peak output of 40 amps
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40; // Peak output of 40 amps

    /* Percent supply gains when we get a Slot 2 */
    configs.Slot1.kP = 0.01;    // An error of 100 rotations per second results in 100% output
    configs.Slot1.kI = 0.04;    // An error of 1 rotation per second increases output by 0.04V every second
    configs.Slot1.kD = 0.00001; // A change of 1 rotation per second squared results in 0.00001 volts output
    configs.Slot1.kV = 0.013;   // Approximately 1.3% for each rotation per second

    configs.MotorOutput.PeakForwardDutyCycle = 0.7;  // Peak output of 70%
    configs.MotorOutput.PeakReverseDutyCycle = -0.7; // Peak output of 70%

    return configs;
  }

  /**
   * Convert int to c string, google itoa
   */
  void itoa(int i, char b[])
  {
    char const digit[] = "0123456789";
    char *p = b;
    if (i < 0)
    {
      *p++ = '-';
      i *= -1;
    }
    int shifter = i;
    do
    { // Move to where representation ends
      ++p;
      shifter = shifter / 10;
    } while (shifter);
    *p = '\0';
    do
    { // Move back, inserting digits as u go
      *--p = digit[i % 10];
      i = i / 10;
    } while (i);
  }

}

void Robot::DisableAllMotors()
{
  for (auto &motor : motors)
  {
    motor.StopMotor();
    motor.Disable();
  }
}

/**
 * Define constants
 * xon & xoff start end commands for RIO
 */
void Robot::RobotInit()
{
  serial.SetTimeout(units::second_t(.1));
  // serial.Reset();

  DisableAllMotors();

  // runs every 1 millisec
  // this is what is actually reading input
  AddPeriodic([this]
              { this->SerialPeriodic(); },
              1_ms);
}

void serial_read_bytes(frc::SerialPort &serial, char *buffer, signed long long buff_size)
{
  do
  {
    auto num_bytes_read = serial.Read(buffer, buff_size);
    buffer = buffer + num_bytes_read;
    buff_size = buff_size - num_bytes_read;
  } while (buff_size != 0);
}


// Format:
// | XON | 4 byte signed opcode | N bytes of opcode specific data
void Robot::SerialPeriodic()
{
  if (!serial_enable)
  {
    return;
  }

  char start_byte = 0;

  if (serial.Read(&start_byte, 1) == 1 && start_byte == XON) // reads 1 byte, if its xon it continues, clear any incomplete buffer and listens to handshake
  {

    int32_t function_number = 0;
    serial.Write(&XON, 1);                                  // response to being on
    serial_read_bytes(serial, (char *)&function_number, 4); // Read function number

    switch (function_number) // choose what to do based on opcode
    {
    case 0:
    {
      struct MotorDataStruct mds = {};
      serial_read_bytes(serial, (char *)&mds, sizeof(MotorDataStruct)); // could be different based on opcode
      switch (mds.call_mode)
      {
      case MotorCallMode::PERCENT: // Percent Output
        motors[mds.motor_number].Set(mds.percent);
        break;

      case MotorCallMode::VELOCITY: // Velocity Output. See https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/cpp/VelocityClosedLoop/src/main/cpp/Robot.cpp
      {
        ctre::phoenix6::controls::VelocityVoltage m_voltageVelocity{0_tps, 0_tr_per_s_sq, true, 0_V, 0, false};
        motors[mds.motor_number].SetControl(m_voltageVelocity.WithVelocity(static_cast<units::angular_velocity::turns_per_second_t>(mds.velocity_turns_per_second)));
        break;
      }
      case MotorCallMode::DISABLE: // Disable Motor Mode
        motors[mds.motor_number].Disable();
        break;

      case MotorCallMode::NEUTRAL_MODE: // Set Neutral Mode
        switch (mds.neutral_mode)
        {
        case MotorNeutralMode::MOTOR_COAST:
          motors[mds.motor_number].SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
          break;
        
        default:
          motors[mds.motor_number].SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
          break;
        }
        break;

      default:
        break;
      }
      // controlling motors
      break;
    }
    }
    serial.Write(&XOFF, 1); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
  }
}

/**
 * eventually maybe not do this
 */
void Robot::RobotPeriodic() {}

/**
 * Zeroing out all motors and enables serial to listen for opcodes
 */
void Robot::AutonomousInit()
{
  // serial.Reset();
  DisableAllMotors();
  serial_enable = true;
}

/**
 * does nothing for now
 */
void Robot::AutonomousPeriodic() {}

/**
 * dont listen to serial, manual control of rover
 */
void Robot::TeleopInit()
{
  serial_enable = false;
}

/**
 * nothing, will eventuall put here
 */
void Robot::TeleopPeriodic() {}

/**
 * Turns off all motors when disabled
 */
void Robot::DisabledInit()
{
  serial_enable = false;
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

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
