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

void Robot::RobotInit() {
  serial.SetTimeout(units::second_t(.1));
  // serial.Reset();
  xon[0] = 0x11;
  xoff[0] = 0x13;

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    motors[i].Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
  }

  AddPeriodic([&] {
    if (serial_enable)
    {
      if (serial.Read(input_buffer, 1) == 1 && input_buffer[0] == 0x11)
      {
        
        serial.Write(xon, 1);
        if (serial.Read(input_buffer, 4))
        {
          int * function_number = (int *)input_buffer;
          switch (*function_number)
          {
            case 0:
              serial.Read(input_buffer, 16);
              int * motor_number   = (int *)(input_buffer);
              int * mode           = (int *)(input_buffer + 4);
              double * outputValue = (double *)(input_buffer + 8);
              motors[*motor_number].Set((ctre::phoenix::motorcontrol::ControlMode)*mode, *outputValue);
          }
          serial.Write(xoff, 1);
        }
      }
    }
  }, 1_ms);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  // serial.Reset();
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    motors[i].Set((ctre::phoenix::motorcontrol::ControlMode)0, 0);
  }
  serial_enable = true;
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  serial_enable = false;
}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  serial_enable = false;
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    motors[i].Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
  }
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
