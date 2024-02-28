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

#include <ctre/phoenix6/signals/SpnEnums.hpp>

using std::cout;
using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

/**
 * Convert int to character, google itoa
 * @param i
 * @param b
 * @return
 */
char *Robot::itoa(int i, char b[])
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
  return b;
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

#pragma pack(1)
struct MotorDataStruct
{
  int32_t motor_number;
  int32_t mode;
  double outputValue;
};

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
      switch (mds.mode)
      {
      case 0: // Percent Output
        motors[mds.motor_number].Set(mds.outputValue);
        break;

      case 15: // Disable Motor Mode
        motors[mds.motor_number].Disable();
        break;

      case 16:
        if (mds.outputValue == 0)
        {
          motors[mds.motor_number].SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
        }else {
          motors[mds.motor_number].SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
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
