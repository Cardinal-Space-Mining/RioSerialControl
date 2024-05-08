// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogPotentiometer.h>
// #include <frc/AnalogInput.h>

#include <chrono>
#include <iostream>
// #include <sys/time.h>
#include <ctime>
#include <cmath>
#include <vector>
#include <deque>
#include <numeric>

#include "LogitechConstants.hpp"

using std::cout; 
using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

/**
 * Disables all motors
*/
void Robot::DisableAllMotors() {
  track_right.Set(0);
  track_left.Set(0);
  trencher.Set(0);
  hopper_belt.Set(0);
  hopper_actuator.Set(0);
}

void Robot::defaultVelocityCfg(TalonFX6 &motor)
{
  configs::TalonFXConfiguration configs{};

  /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
  configs.Slot0.kP = 0.11;   // An error of 1 rotation per second results in 2V output
  configs.Slot0.kI = 0.5;    // An error of 1 rotation per second increases output by 0.5V every second
  configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.0001 volts output
  configs.Slot0.kV = 0.12;   // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

  motor.GetConfigurator().Apply(configs);
}

void Robot::ConfigTracks()
{
  configs::TalonFXConfiguration configs{};

  /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
  configs.Slot0.kP = 0.11;   // An error of 1 rotation per second results in 2V output
  configs.Slot0.kI = 0.5;    // An error of 1 rotation per second increases output by 0.5V every second
  configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.0001 volts output
  configs.Slot0.kV = 0.12;   // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

  configs.CurrentLimits.StatorCurrentLimitEnable = false;
  
  track_left.GetConfigurator().Apply(configs);
  track_right.GetConfigurator().Apply(configs);
}

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

/**
 * Define constants
 * xon & xoff start end commands for RIO
 */
void Robot::RobotInit() {

  trencher.SetInverted(true);
  hopper_belt.SetInverted(false);
  track_right.SetInverted(true);

  serial.SetTimeout(units::second_t(.1));
  // serial.Reset();
  xon[0] = 0x13;
  xoff[0] = 0x11;

  DisableAllMotors();

  // serial.Reset();

  // runs every 1 millisec
  // this is what is actually reading input


  AddPeriodic([&] {
    // start mining lower actuators
    if (is_mining && !time_set) {

      double pos = hopper_actuator_pot.Get();

      if (pos > mining_depth) {
        hopper_actuator.Set(0.75);
      } else {
        hopper_actuator.Set(0.0);
        time_set = true;
        start_time = std::chrono::system_clock::now();  
        if (serial_enable) {
          serial.Write(xoff, 1); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
        }
      }
    }

    // keep track of mining time, has reached run time?
    if (is_mining && time_set && !finished_cycle) {
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_time);
      if (duration.count() > mining_run_time && !serial_enable) {
        StopMining();
      }

      if (time_set) {
        // drivetrain motion settings
        ctre::phoenix6::controls::VelocityVoltage drivetrain_velo {TRACKS_MINING_MAX_VELO, 1_tr_per_s_sq, false, 0_V, 0, false};
        track_left.SetControl(drivetrain_velo);
        track_right.SetControl(drivetrain_velo);
      }
    }

    // raise hopper from mining position
    if (finished_cycle && is_mining) {
      
      double pos = hopper_actuator_pot.Get();

      track_right.Set(0);
      track_left.Set(0);

      if (pos < mining_to_offload_depth) {
        hopper_actuator.Set(-0.75);
      } else {
        is_mining = false;
        finished_cycle = false;
        DisableAllMotors();
        if (serial_enable) {
          serial.Write(xoff, 1); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
        }
      } 
    }
  }, 20_ms);

  // AddPeriodic([&] {
  //   if (is_mining && time_set && !finished_cycle) {

  //     if (on_off) {
  //       cout << "starting hopper" << endl;
  //       ctre::phoenix6::controls::VelocityVoltage hopper_belt_velo {HOPPER_BELT_MAX_MINING_VELO * -1.0, 1_tr_per_s_sq, false, 0_V, 0, false};
  //       hopper_belt.SetControl(hopper_belt_velo);
  //       on_off = !on_off;
  //     } else {
  //       cout << "stopping hopper" << endl;
  //       hopper_belt.Set(0);
  //       on_off = !on_off;
  //     }
  // }
  // }, 350_ms);
  
  AddPeriodic([&] {
    // move to offload position
    if (is_offload && !is_offload_pos) {
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_move_time_off);

      if (duration.count() > offload_move_time) {
        is_offload_pos = true;
        track_left.Set(0);
        track_right.Set(0);
      } else {
        ctre::phoenix6::controls::VelocityVoltage drivetrain_velo {TRACKS_MAX_VELO * -0.25, 1_tr_per_s_sq, false, 0_V, 0, false};
        track_left.SetControl(drivetrain_velo);
        track_right.SetControl(drivetrain_velo);
      }
    }

    // start offload raise actuators
    if (is_offload && is_offload_pos && !time_set) {
      double pos = hopper_actuator_pot.Get();

      if (pos < offload_depth) {
        hopper_actuator.Set(-1.0);
      } else {
        hopper_actuator.Set(0.0);
        time_set = true;
        start_time = std::chrono::system_clock::now();
        ctre::phoenix6::controls::VelocityDutyCycle hopper_belt_velo = -1.0 * HOPPER_BELT_MAX_VELO; 
        hopper_belt.SetControl(hopper_belt_velo); 
      }
    }

    // keep track of offload time, has reached run time?
    if (is_offload && time_set && !finished_cycle) {
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_time);
      if (duration.count() > offload_run_time) {
        hopper_actuator.Set(0);
        time_set = false;
        StopOffload();
      }
    }

    // lower hopper from offload postition
    if (is_offload && finished_cycle) {

      double pos = hopper_actuator_pot.Get();

      if (pos > reg_traversal_depth) {
        hopper_actuator.Set(1.0);
      } else {
        is_offload_pos = false;
        is_offload = false;
        finished_cycle = false;
        time_set = false;
        DisableAllMotors();
        if (serial_enable) {
          serial.Write(xoff, 1); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
        }
      }
    }
  }, 20_ms);

  // AddPeriodic([&] {
  //   // TODO needs to be moved to mining algorithm bc its in while loop
  //   // moving average current check
  //   if (is_mining) {
  //     // TODO get current from trencher
  //     double newCurrent = 1.0;
  //     motorDataList.push_back(newCurrent);
  //     double currentAverage = 0;
  //     if (motorDataList.size() > movingAvgRange) {
  //       motorDataList.pop_front();
  //     }
  //     currentAverage = std::accumulate(motorDataList.begin(), motorDataList.end(), 0.0);
  //     currentAverage /= motorDataList.size();
  //     trenchAvgCurrent = currentAverage;
  //     if (trenchAvgCurrent > safteyAvgCurrent) {
  //       // DisableAllMotors();
  //       hopper_belt.Set(0);
  //       track_left.Set(0);
  //       track_right.Set(0);
  //       // trencher.Set(0);
  //       while(true) {
  //         double pos = hopper_actuator_pot.Get();
  //         if (pos < mining_to_offload_depth) {
  //           hopper_actuator.Set(-0.5);
  //         } 
  //         else {
  //           hopper_actuator.Set(0.0);
  //           break;
  //         }
  //         std::this_thread::sleep_for(std::chrono::milliseconds(250));
  //       }
  //       // lift trencher 
  //       // drive back
  //       // start mining again
  //       trenchAvgCurrent = 0;
  //       motorDataList.clear();
  //     }
  //   }
  // }, 100_ms);

}

/**
 * eventually maybe not do this
 */
void Robot::RobotPeriodic() {}

/**
 * Zeroing out all motors and enables serial to listen for opcodes
 */
void Robot::AutonomousInit() {
  DisableAllMotors();
  serial_enable = true;
  is_mining = false;
  is_offload = false;
  finished_cycle = false;
  time_set = false;
}

/**
 * does nothing for now
 */
void Robot::AutonomousPeriodic() {
    AddPeriodic([&] {
    if (serial_enable)
    {
      if (serial.Read(input_buffer, 1) == 1 && input_buffer[0] == 0x13) //reads 1 byte, if its xon it continues, clear any incomplete buffer and listens to handshake
      {
        serial.Write(xon, 1); // response to being on
        if (serial.Read(input_buffer, 4)) // in corresponding to opcode 4 byte int
        {
          int function_number = *reinterpret_cast<int*>(input_buffer);
          uint8_t result;
          switch (function_number) // choosing what to do based on opcode
          {
            case 0: //spinning up motor with params id and output
            {
              serial.Read(input_buffer, 12); // could be different based on opcode

              int motor_number = *reinterpret_cast<int*>(input_buffer);
              double output_value = *reinterpret_cast<double*>(input_buffer + 4);
              (*motors[motor_number]).Set(output_value);
              break;
            }
            case 1: // start autonomous mining
              StartMining();
              break;
            case 2: // stop autonomous mining
              StopMining();
              break;
            case 3: // start autonomous offload
              StartOffload();
              break;
            case 4: // stop autonomous offload
              StopOffload();
              break;
          }
          // serial.Write(result, 1);
          // serial.Write(xoff, 1); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
        }
      }
    }
  }, 1_ms);
  
  AddPeriodic([&] {
    if (is_mining && time_set && !finished_cycle) {

      if (on_off) {
        cout << "starting hopper" << endl;
        ctre::phoenix6::controls::VelocityVoltage hopper_belt_velo {HOPPER_BELT_MAX_MINING_VELO * -1.0, 1_tr_per_s_sq, false, 0_V, 0, false};
        hopper_belt.SetControl(hopper_belt_velo);
        on_off = !on_off;
      } else {
        cout << "stopping hopper" << endl;
        hopper_belt.Set(0);
        on_off = !on_off;
      }
  }
  }, 350_ms);
}

/**
 * dont listen to serial, manual control of rover
 */
void Robot::TeleopInit() {
  DisableAllMotors();
  serial_enable = false;
  is_mining = false;
  is_offload = false;
  finished_cycle = false;
  time_set = false;
}

static constexpr long double PI = 3.14159265358979323846;

uint8_t Robot::getMovingAvg() {
  double newCurrent = 1.0;
  motorDataList.push_back(newCurrent);
  double currentAverage = 0;
  if (motorDataList.size() > movingAvgRange) {
    motorDataList.pop_front();
  }
  currentAverage = std::accumulate(motorDataList.begin(), motorDataList.end(), 0.0);
  currentAverage /= motorDataList.size();
  trenchAvgCurrent = currentAverage;
  if (trenchAvgCurrent > safteyAvgCurrent) {
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t Robot::StartMining() {

  if (!is_mining && !is_offload) {

    DisableAllMotors();
    time_set = false;

    // double belt_percentage = 1.0;
    
    ctre::phoenix6::controls::VelocityVoltage trencher_velo {TRENCHER_MAX_VELO, 5_tr_per_s_sq, false, 0_V, 0, false};
    trencher.SetControl(trencher_velo);

    is_mining = true;

    start_time = std::chrono::system_clock::now();

    // auto start_time = std::chrono::system_clock::now();
    // while (true) {
    //   auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_time);
    //   if (duration.count() > mining_run_time) {
    //     StopMining();
    //     break;
    //   } 
    //   else {
    //     ctre::phoenix6::controls::VelocityDutyCycle hopper_belt_velo = -1.0 * HOPPER_BELT_MAX_MINING_VELO; 
    //     hopper_belt.SetControl(hopper_belt_velo); 

    //     std::this_thread::sleep_for(std::chrono::milliseconds(hopper_belt_mine_run_time));

    //     hopper_belt.Set(0);

    //     if (getMovingAvg() == 1) {
    //       hopper_belt.Set(0);
    //       track_left.Set(0);
    //       track_right.Set(0);
    //       // trencher.Set(0);
    //       while(true) {
    //         double pos = hopper_actuator_pot.Get();
    //         if (pos < mining_to_offload_depth) {
    //           hopper_actuator.Set(-0.5);
    //         } 
    //         else {
    //           hopper_actuator.Set(0.0);
    //           break;
    //         }
    //         std::this_thread::sleep_for(std::chrono::milliseconds(250));
    //       }
    //       // lift trencher 
    //       // drive back
    //       // start mining again
    //       trenchAvgCurrent = 0;
    //       motorDataList.clear();
    //       track_left.SetControl(drivetrain_velo);
    //       track_right.SetControl(drivetrain_velo);
    //     }
    //   }

    //   std::this_thread::sleep_for(std::chrono::milliseconds(hopper_belt_mine_wait_time));
    // }

  } else {
    if (is_mining) {
      return 1;
    } else {
      return 3;
    }
  }

  return 0;
}

uint8_t Robot::StopMining() {

  if (is_mining && !is_offload) {

    DisableAllMotors();

    ctre::phoenix6::controls::VelocityVoltage trencher_velo {TRENCHER_MAX_VELO, 5_tr_per_s_sq, false, 0_V, 0, false};
    trencher.SetControl(trencher_velo);

    finished_cycle = true;

  } else {
    if (!is_mining) {
      return 1;
    } else {
      return 3;
    }
  }

  return 0;
}

uint8_t Robot::StartOffload() {

  if (!is_offload && !is_mining) {

    DisableAllMotors();
    time_set = false;
    is_offload_pos = false;

    is_offload = true;
    start_time = std::chrono::system_clock::now();
    start_move_time_off = std::chrono::system_clock::now();

    // while (true) {
    //   double pos = hopper_actuator_pot.Get();

    //   if (pos < offload_depth) {
    //     hopper_actuator.Set(-0.5);
    //   } else {
    //     hopper_actuator.Set(0.0);
    //     break;
    //   }

    //   std::this_thread::sleep_for(std::chrono::milliseconds(250));
    // }

    // // hopper belt
    // ctre::phoenix6::controls::VelocityDutyCycle hopper_belt_velo = -1.0 * HOPPER_BELT_MAX_VELO; 
    // hopper_belt.SetControl(hopper_belt_velo); 

    // track_left.Set(.01);
    // track_right.Set(.01);

    // // for timed offloading uncomment this block
    // if (!serial_enable) {
    //   std::this_thread::sleep_for(std::chrono::seconds(6));
    //   StopOffload();
    // }

    // auto start_time = std::chrono::system_clock::now();
    // while (true) {
    //   auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_time);
    //   if (duration.count() > 6) {
    //     StopOffload();
    //     break;
    //   }
    // }

    return 0;
  } else {
    if (is_offload) {
      return 1;
    } else {
      return 3;
    }
  }
}

uint8_t Robot::StopOffload() {

  if ((is_offload && !is_mining) || true) {

    DisableAllMotors();

    finished_cycle = true;

    return 0;
  } else {
    if (!is_offload) {
      return 2;
    } else {
      return 3;
    }
  }
}


void Robot::TrencherControl()
{
  double belt_percentage = logitech.GetRawAxis(LogitechConstants::RIGHT_TRIGGER);
  if (logitech.GetRawButton(LogitechConstants::RB))
  {
    belt_percentage = -1.0 * belt_percentage;
  }

  ctre::phoenix6::controls::VelocityVoltage trencherVelocity{TRENCHER_MAX_VELO * belt_percentage, 5_tr_per_s_sq, false, 0_V, 0, false};
  trencher.SetControl(trencherVelocity);
}

void Robot::HopperControl()
{
  // Control Hopper Belt
  double belt_percentage = -logitech.GetRawAxis(LogitechConstants::LEFT_TRIGGER);
  if (logitech.GetRawButton(LogitechConstants::LB))
  {
    belt_percentage = -1.0 * belt_percentage;
  }

  ctre::phoenix6::controls::VelocityVoltage belt_velo{HOPPER_BELT_MAX_VELO * belt_percentage, 5_tr_per_s_sq, false, 0_V, 0, false};
  hopper_belt.SetControl(belt_velo);

  // Control Hopper Actuator
  double actuator_power = -logitech.GetRawAxis(LogitechConstants::RIGHT_JOY_Y);
  hopper_actuator.Set(-actuator_power);

}

void Robot::DriveTrainControl()
{
  using namespace LogitechConstants;

  ctre::phoenix6::controls::VelocityVoltage m_voltageVelocity{0_tps, 1_tr_per_s_sq, false, 0_V, 0, false};

  const double stick_x = logitech.GetRawAxis(LEFT_JOY_X);
  const double stick_y = -logitech.GetRawAxis(LEFT_JOY_Y);

  const double theta = std::atan2(stick_x, stick_y);
  double magnitude = std::sqrt(std::pow(stick_x, 2) + std::pow(stick_y, 2));
  if (magnitude < 0.1)
  {
    magnitude = 0;
  }

  // if (logitech.GetRawButton(LogitechConstants::BUTTON_X))
  // {
  //   drive_power_scale_factor = 1;
  // }

  // if (logitech.GetRawButton(LogitechConstants::BUTTON_Y))
  // {
  //   drive_power_scale_factor = 0.7;
  // }

  // if (logitech.GetRawButton(LogitechConstants::BUTTON_B))
  // {
  //   drive_power_scale_factor = 0.3;
  // }

  auto r_velo = drive_power_scale_factor * TRACKS_MAX_VELO * magnitude * std::cos(theta + (PI / 4));
  auto l_velo = drive_power_scale_factor * TRACKS_MAX_VELO * magnitude * std::cos(theta - (PI / 4));

  track_right.SetControl(m_voltageVelocity.WithVelocity(r_velo));
  track_left.SetControl(m_voltageVelocity.WithVelocity(l_velo));
}

void Robot::TeleopControl() {

  if (logitech.GetRawButton(LogitechConstants::BUTTON_A)) {
    StartMining();
  }

  if (logitech.GetRawButton(LogitechConstants::BUTTON_B)) {
    StartOffload();
  }

}


/**
 * nothing, will eventuall put here
 */
void Robot::TeleopPeriodic() {
  if (is_mining || is_offload)
  {
    return;
  }

  AddPeriodic([&] {
    cout << "attempting this" << endl;
    if (is_mining && time_set && !finished_cycle) {

      if (on_off) {
        ctre::phoenix6::controls::VelocityVoltage hopper_belt_velo {HOPPER_BELT_MAX_MINING_VELO * -1.0, 1_tr_per_s_sq, false, 0_V, 0, false};
        hopper_belt.SetControl(hopper_belt_velo);
        on_off = !on_off;
      } else {
        hopper_belt.Set(0);
        on_off = !on_off;
      }
  }
  }, 450_ms);
  
  this->DriveTrainControl();
  this->HopperControl();
  this->TrencherControl();
  this->TeleopControl();

  serial_enable = false;
}

/**
 * Turns off all motors when disabled
 */
void Robot::DisabledInit() {
  DisableAllMotors();
  serial_enable = false;
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
