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

using std::cout; using std::endl;
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
char* Robot::itoa(int i, char b[]){
    char const digit[] = "0123456789";
    char* p = b;
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    int shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return b;
}

void Robot::DisableAllMotors(){
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
void Robot::RobotInit() {
  serial.SetTimeout(units::second_t(.1));
  // serial.Reset();
  xon[0] = 0x11;
  xoff[0] = 0x13;

  DisableAllMotors();

  // runs every 1 millisec
  // this is what is actually reading input
  AddPeriodic([&] {
    if (serial_enable)
    {
      if (serial.Read(input_buffer, 1) == 1 && input_buffer[0] == 0x11) //reads 1 byte, if its xon it continues, clear any incomplete buffer and listens to handshake
      {
        
        serial.Write(xon, 1); // response to being on
        if (serial.Read(input_buffer, 4)) // in corresponding to opcode 4 byte int
        {
          int * function_number = (int *)input_buffer;
          switch (*function_number) // choosing what to do based on opcode
          {
            case 0:
              serial.Read(input_buffer, 16); // could be different based on opcode
              int * motor_number   = (int *)(input_buffer);         // which motor for command
              int * mode           = (int *)(input_buffer + 4);     // velocity/motion magic (enums)
              double * outputValue = (double *)(input_buffer + 8);  // percent/value
              motors[*motor_number].Set(*outputValue); // controlling motors
              break;
          }
          serial.Write(xoff, 1); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
        }
      }
    }
  }, 1_ms);
}

/**
 * eventually maybe not do this
 */
void Robot::RobotPeriodic() {}

/**
 * Zeroing out all motors and enables serial to listen for opcodes
 */
void Robot::AutonomousInit() {
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
void Robot::TeleopInit() {
  serial_enable = false;
}

/**
 * nothing, will eventuall put here
 */
void Robot::TeleopPeriodic() {}

/**
 * Turns off all motors when disabled
 */
void Robot::DisabledInit() {
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
int main() {
  return frc::StartRobot<Robot>();
}
#endif
