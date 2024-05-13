// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogPotentiometer.h>

#include <frc/drive/DifferentialDrive.h>
// #include <frc/AnalogInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>

#include <chrono>
#include <iostream>
// #include <sys/time.h>
#include <ctime>
#include <cmath>
#include <vector>
#include <deque>
#include <numeric>


using std::cout; 
using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

static constexpr long double PI = 3.14159265358979323846;

namespace util {

	/** Convert int to character, google itoa
	 * @param i
	 * @param b
	 * @return */
	char* itoa(int i, char b[])
	{
		char const digit[] = "0123456789";
		char* p = b;
		if(i<0){
			*p++ = '-';
			i *= -1;
		}
		int shifter = i;
		do { //Move to where representation ends
			++p;
			shifter = shifter/10;
		} while(shifter);
		*p = '\0';
		do { //Move back, inserting digits as u go
			*--p = digit[i%10];
			i = i/10;
		} while(i);
		return b;
	}

	frc::DifferentialDrive::WheelSpeeds computeWheelScalars(double x, double y, double deadzone)
	{
		const double augmented_angle = std::atan2(x, y) + (PI / 4);	// x and y are inverted to make a CW "heading" angle
		double magnitude = std::sqrt(x*x + y*y);
		if (magnitude < 0.1) return { 0.0, 0.0 };

		return {
			magnitude * std::cos(augmented_angle),
			magnitude * std::sin(augmented_angle)		// this is the same as cos("raw theta" - pi/4) like from the original code
		};
	}

	void add_fx6_dbg_info(wpi::SendableBuilder& builder, TalonFX6& motor, std::string prefix)
	{
		builder.AddDoubleProperty(prefix + "/duty_cycle", [&] { return motor.Get(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/voltage", [&] { return motor.GetMotorVoltage().GetValue().value(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/position", [&] { return motor.GetPosition().GetValue().value(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/current", [&] { return motor.GetStatorCurrent().GetValue().value(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/temp", [&] { return motor.GetDeviceTemp().GetValue().value(); }, nullptr);
	}
	void add_fx5_dbg_info(wpi::SendableBuilder& builder, TalonFX5& motor, std::string prefix)
	{
		builder.AddDoubleProperty(prefix + "/duty_cycle", [&] { return motor.Get(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/voltage", [&] { return motor.GetMotorOutputVoltage(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/position", [&] { return motor.GetSelectedSensorPosition(0 /*Primary Closed Loop*/); }, nullptr);
		builder.AddDoubleProperty(prefix + "/current", [&] { return motor.GetStatorCurrent(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/temp", [&] { return motor.GetTemperature(); }, nullptr);
	}

}

void Robot::State::InitSendable(wpi::SendableBuilder& builder)
{
	builder.AddBooleanProperty("serial_enabled", [this](){return this->serial_enabled;}, nullptr);
	builder.AddBooleanProperty("mining_enabled", [this](){return this->mining_enabled;}, nullptr);
	builder.AddBooleanProperty("offload_enabled", [this](){return this->offload_enabled;}, nullptr);
	builder.AddBooleanProperty("mining_complete", [this](){return this->mining_complete;}, nullptr);
	builder.AddBooleanProperty("hopper_enabled", [this](){return this->hopper_enabled;}, nullptr);
	builder.AddBooleanProperty("offload_traversal_reached", [this](){return this->offload_traversal_reached;}, nullptr);
	builder.AddBooleanProperty("mining_lowered_hopper", [this](){return this->mining_lowered_hopper;}, nullptr);
	builder.AddDoubleProperty("driving_speed_scalar", [this](){return this->driving_speed_scalar;}, nullptr);
};



Robot::Robot() /*: telemetry_sender{ "Robot" }*/ {}
Robot::~Robot() {}


// TELEMETRY
void Robot::InitSendable(wpi::SendableBuilder& builder)
{
	util::add_fx6_dbg_info(builder, track_right, "TrackRight");
	util::add_fx6_dbg_info(builder, track_left, "TrackLeft");
	util::add_fx6_dbg_info(builder, trencher, "Trencher");
	util::add_fx6_dbg_info(builder, hopper_belt, "HopperBelt");
	util::add_fx5_dbg_info(builder, hopper_actuator, "HopperActuator");
	this->state.InitSendable(builder);
}



// -------- Moving Average -------------
uint8_t Robot::get_moving_avg()
{
	double newCurrent = 1.0;
	motorDataList.push_back(newCurrent);
	double currentAverage = 0;

	if(motorDataList.size() > movingAvgRange) motorDataList.pop_front();

	currentAverage = std::accumulate(motorDataList.begin(), motorDataList.end(), 0.0);
	currentAverage /= motorDataList.size();
	trenchAvgCurrent = currentAverage;

	if(trenchAvgCurrent > avg_current_thresh) return 1;
	else return 0;
}



// --------- Setup/Helpers ----------
void Robot::configure_motors()
{
	configs::TalonFXConfiguration generic_config{}/*, tracks_config{}*/;

	/* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
	generic_config.Slot0.kP = 0.11;   // An error of 1 rotation per second results in 2V output
	generic_config.Slot0.kI = 0.5;    // An error of 1 rotation per second increases output by 0.5V every second
	generic_config.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.0001 volts output
	generic_config.Slot0.kV = 0.12;   // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

	// tracks_config.CurrentLimits.StatorCurrentLimitEnable = false;
	generic_config.CurrentLimits.StatorCurrentLimitEnable = false;

	track_left.GetConfigurator().Apply(generic_config);
	track_right.GetConfigurator().Apply(generic_config);

	hopper_belt.GetConfigurator().Apply(generic_config);
	trencher.GetConfigurator().Apply(generic_config);

	// TODO: disable hopper actuator current limit

	trencher.SetInverted(true);
	hopper_belt.SetInverted(false);
	track_right.SetInverted(true);
}

void Robot::disable_motors()
{
	track_right.Set(0);
	track_left.Set(0);
	trencher.Set(0);
	hopper_belt.Set(0);
	hopper_actuator.Set(0);
}



// ------------- Mining/Offload init and shutdown --------------
void Robot::mining_init()
{
	if(!this->state.mining_enabled && !this->state.offload_enabled)
	{
		this->disable_motors();
		this->state.mining_lowered_hopper = false;

		ctre::phoenix6::controls::VelocityVoltage trencher_velo{ Robot::TRENCHER_MAX_VELO, 5_tr_per_s_sq, false, 0_V, 0, false };
		trencher.SetControl(trencher_velo);

		this->state.mining_enabled = true;
		this->state.auto_operation_start_time = std::chrono::system_clock::now();
	}
}

void Robot::mining_shutdown()
{
	if (this->state.mining_enabled && !this->state.offload_enabled)
	{
		this->disable_motors();

		ctre::phoenix6::controls::VelocityVoltage trencher_velo{ Robot::TRENCHER_MAX_VELO, 5_tr_per_s_sq, false, 0_V, 0, false };
		trencher.SetControl(trencher_velo);

		this->state.mining_complete = true;
	}

}

void Robot::offload_init()
{
	if(!this->state.offload_enabled && !this->state.mining_enabled)
	{
		this->disable_motors();

		this->state.mining_lowered_hopper = false;
		this->state.offload_traversal_reached = false;

		this->state.offload_enabled = true;
		this->state.auto_operation_start_time = std::chrono::system_clock::now();
		this->state.offload_traversal_start_time = std::chrono::system_clock::now();
	}
}

void Robot::offload_shutdown()
{
	if((this->state.offload_enabled && !this->state.mining_enabled) || true)
	{
		this->disable_motors();
		this->state.mining_complete = true;
	}
}





void Robot::periodic_handle_serial_control()
{
	if (this->state.serial_enabled)
	{
		if (serial.Read(input_buffer, 1) == 1 && input_buffer[0] == 0x13) //reads 1 byte, if its xon it continues, clear any incomplete buffer and listens to handshake
		{
			serial.Write(xon, 1); // response to being on
			if (serial.Read(input_buffer, 4)) // in corresponding to opcode 4 byte int
			{
				int function_number = *reinterpret_cast<int*>(input_buffer);
				// uint8_t result;
				switch (function_number) // choosing what to do based on opcode
				{
					case 0: //spinning up motor with params id and output
					{
						serial.Read(input_buffer, 12); // could be different based on opcode

						int motor_number = *reinterpret_cast<int*>(input_buffer);
						double output_value = *reinterpret_cast<double*>(input_buffer + 4);
						motors[motor_number]->Set(output_value);
						break;
					}
					case 1: // start autonomous mining
						mining_init();
						break;
					case 2: // stop autonomous mining
						mining_shutdown();
						break;
					case 3: // start autonomous offload
						offload_init();
						break;
					case 4: // stop autonomous offload
						offload_shutdown();
						break;
				}
				// serial.Write(result, 1);
				// serial.Write(xoff, 1); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
			}
		}
	}
}

void Robot::periodic_handle_mining()
{
	// start mining lower actuators
	if(this->state.mining_enabled && !this->state.mining_lowered_hopper)
	{
		double pos = hopper_actuator_pot.Get();

		if(pos > Robot::MINING_POT_VALUE) {
			hopper_actuator.Set(0.75);
		} else {
			hopper_actuator.Set(0.0);
			this->state.mining_lowered_hopper = true;
			this->state.auto_operation_start_time = std::chrono::system_clock::now();  
			if (this->state.serial_enabled) {
				serial.Write(xoff, 1); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
			}
		}
	}

	// keep track of mining time, has reached run time?
	if(this->state.mining_enabled && this->state.mining_lowered_hopper && !this->state.mining_complete)
	{
		auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - this->state.auto_operation_start_time);
		if(duration.count() > Robot::MINING_RUN_TIME_SECONDS && !this->state.serial_enabled) {
			this->mining_shutdown();
		}

		if(this->state.mining_lowered_hopper) {
			// drivetrain motion settings
			ctre::phoenix6::controls::VelocityVoltage drivetrain_velo { Robot::TRACKS_MINING_MAX_VELO, 1_tr_per_s_sq, false, 0_V, 0, false };
			track_left.SetControl(drivetrain_velo);
			track_right.SetControl(drivetrain_velo);
		}
	}

    // raise hopper from mining position
    if(this->state.mining_complete && this->state.mining_enabled)
	{  
		double pos = hopper_actuator_pot.Get();

		track_right.Set(0);
		track_left.Set(0);

		if (pos < Robot::AUTO_TRANSPORT_POT_VALUE) {
			hopper_actuator.Set(-0.75);
		} else {
			this->state.mining_enabled = false;
			this->state.mining_complete = false;
			this->disable_motors();
			if (this->state.serial_enabled) {
				serial.Write(xoff, 1); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
			}
		} 
    }
}

void Robot::periodic_handle_offload()
{
	// move to offload position
	if(this->state.offload_enabled && !this->state.offload_traversal_reached)
	{
		auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - this->state.offload_traversal_start_time);
		if ((this->state.serial_enabled && duration.count() > Robot::AUTO_OFFLOAD_BACKUP_TIME_SECONDS) || 
			(!this->state.serial_enabled && duration.count() > Robot::TELE_OFFLOAD_BACKUP_TIME_SECONDS)) 
		{
			this->state.offload_traversal_reached = true;
			track_left.Set(0);
			track_right.Set(0);
		}
		// if (duration.count() > TELE_OFFLOAD_BACKUP_TIME_SECONDS) {
		//   this->state.offload_traversal_reached = true;
		//   track_left.Set(0);
		//   track_right.Set(0);
		// } 
		else
		{
			ctre::phoenix6::controls::VelocityVoltage drivetrain_velo{ Robot::TRACKS_MAX_VELO * -0.25, 1_tr_per_s_sq, false, 0_V, 0, false };

			track_left.SetControl(drivetrain_velo);
			track_right.SetControl(drivetrain_velo);
		}
	}

    // start offload raise actuators
    if(this->state.offload_enabled && this->state.offload_traversal_reached && !this->state.mining_lowered_hopper)
	{
		double pos = hopper_actuator_pot.Get();

		if (pos < Robot::OFFLOAD_POT_VALUE) {
			hopper_actuator.Set(-1.0);
		} else {
			hopper_actuator.Set(0.0);
			this->state.mining_lowered_hopper = true;
			this->state.auto_operation_start_time = std::chrono::system_clock::now();

			ctre::phoenix6::controls::VelocityDutyCycle hopper_belt_velo = Robot::HOPPER_BELT_MAX_VELO * -1.0;
			hopper_belt.SetControl(hopper_belt_velo);
		}
    }

    // keep track of offload time, has reached run time?
    if(this->state.offload_enabled && this->state.mining_lowered_hopper && !this->state.mining_complete) {
		auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - this->state.auto_operation_start_time);
		if (duration.count() > Robot::OFFLOAD_TOTAL_RUN_TIME) {
			hopper_actuator.Set(0);
			this->state.mining_lowered_hopper = false;
			this->offload_shutdown();
		}
    }

    // lower hopper from offload postition
    if(this->state.offload_enabled && this->state.mining_complete)
	{
		double pos = hopper_actuator_pot.Get();

		if (pos > Robot::TRAVERSAL_POT_VALUE) {
			hopper_actuator.Set(1.0);
		} else {
			this->state.offload_traversal_reached = false;
			this->state.offload_enabled = false;
			this->state.mining_complete = false;
			this->state.mining_lowered_hopper = false;
			this->disable_motors();
			if (this->state.serial_enabled) {
				serial.Write(xoff, 1); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
			}
		}
    }
}



// ------------------ Periodic Handlers --------------------
void Robot::periodic_handle_teleop_input()
{
	// ------- DRIVE CONTROL ------------
	{
		// handle driving speed updates
		if(logitech.GetRawButtonPressed(Robot::TELEOP_LOW_SPEED_BUTTON_IDX)) this->state.driving_speed_scalar = Robot::DRIVING_LOW_SPEED_SCALAR;
		if(logitech.GetRawButtonPressed(Robot::TELEOP_MEDIUM_SPEED_BUTTON_IDX)) this->state.driving_speed_scalar = Robot::DRIVING_MEDIUM_SPEED_SCALAR;
		if(logitech.GetRawButtonPressed(Robot::TELEOP_HIGH_SPEED_BUTTON_IDX)) this->state.driving_speed_scalar = Robot::DRIVING_HIGH_SPEED_SCALAR;

		ctre::phoenix6::controls::VelocityVoltage vel_command{0_tps, 1_tr_per_s_sq, false, 0_V, 0, false};

		const double
			stick_x = logitech.GetRawAxis(Robot::TELEOP_DRIVE_X_AXIS_IDX),
			stick_y = -logitech.GetRawAxis(Robot::TELEOP_DRIVE_Y_AXIS_IDX);	// forward y is positive

		auto track_speeds = util::computeWheelScalars(stick_x, stick_y, Robot::DRIVING_DEADZONE_SCALAR);

		// set drive velocities
		track_right.SetControl(
			vel_command.WithVelocity(
				TRACKS_MAX_VELO * this->state.driving_speed_scalar * track_speeds.right
			));
		track_left.SetControl(
			vel_command.WithVelocity(
				TRACKS_MAX_VELO * this->state.driving_speed_scalar * track_speeds.right
			));
	}
	// -------- BELT CONTROL -------------
	{
		double trencher_speed = logitech.GetRawAxis(Robot::TELEOP_TRENCHER_SPEED_AXIS_IDX);
		if (logitech.GetRawButton(Robot::TELEOP_TRENCHER_INVERT_IDX)) trencher_speed *= -1.0;

		// set trencher velocity
		ctre::phoenix6::controls::VelocityVoltage vel_command{ (Robot::TRENCHER_MAX_VELO * trencher_speed), 5_tr_per_s_sq, false, 0_V, 0, false };
		trencher.SetControl(vel_command);
	}
	// -------- HOPPER CONTROL -------------
	{
		double hopper_belt_speed = -logitech.GetRawAxis(Robot::TELEOP_HOPPER_SPEED_AXIS_IDX);
		if (logitech.GetRawButton(Robot::TELEOP_HOPPER_INVERT_IDX)) hopper_belt_speed *= -1.0;

		// set hopper belt
		ctre::phoenix6::controls::VelocityVoltage belt_velo{ (HOPPER_BELT_MAX_VELO * hopper_belt_speed), 5_tr_per_s_sq, false, 0_V, 0, false };
		hopper_belt.SetControl(belt_velo);

		// set actutor power
		hopper_actuator.Set(
			logitech.GetRawAxis(Robot::TELEOP_HOPPER_ACTUATE_AXIS_IDX)
		);
	}
	// ---------- TELEAUTO CONTROl ----------
	{
		if(logitech.GetPOV(0) == Robot::TELEAUTO_MINING_INIT_POV) {		// dpad top
			this->mining_init();
		}
		if(logitech.GetPOV(0) == Robot::TELEAUTO_OFFLOAD_INIT_POV) {	// dpad bottom
			this->offload_init();
		}
	}
}





// -------------- TimedRobot Overrides --------------
void Robot::RobotInit()
{
	// setup serial
	xon[0] = 0x13;
	xoff[0] = 0x11;
	serial.SetTimeout(units::second_t(.1));
	// serial.Reset();

	// setup motors
	this->configure_motors();
	this->disable_motors();

	// handle serial updates at a 1 ms interval - this rate may need to be lowered if this causes issues
	this->AddPeriodic(std::bind(&Robot::periodic_handle_serial_control, this), 20_ms);

	// TODO: fix this with correct timing control
	this->AddPeriodic([&] {
		if (this->state.mining_enabled && this->state.mining_lowered_hopper && !this->state.mining_complete) {
			if (this->state.hopper_enabled) {
				ctre::phoenix6::controls::VelocityVoltage hopper_belt_velo {HOPPER_BELT_MAX_MINING_VELO * -1.0, 1_tr_per_s_sq, false, 0_V, 0, false};
				hopper_belt.SetControl(hopper_belt_velo);
				this->state.hopper_enabled = !this->state.hopper_enabled;
			} else {
				hopper_belt.Set(0);
				this->state.hopper_enabled = !this->state.hopper_enabled;
			}
		}
	}, 1000_ms);

	// this->telemetry_sender.putData(this);
	frc::SmartDashboard::PutData("robot", this);

	if(this->IsReal()) {
		frc::DataLogManager::Start();	// setup offline logging
		frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());	// log driverstation inputs
	}
	
}

void Robot::RobotPeriodic()
{
	// run periodic control
	this->periodic_handle_mining();
	this->periodic_handle_offload();

	// this->telemetry_sender.updateValues();
}


void Robot::AutonomousInit()
{
	this->disable_motors();

	this->state.serial_enabled = true;
	this->state.mining_enabled = false;
	this->state.offload_enabled = false;
	this->state.mining_complete = false;
	this->state.mining_lowered_hopper = false;
}

void Robot::AutonomousPeriodic() {}


void Robot::TeleopInit()
{
	this->disable_motors();

	this->state.serial_enabled = false;
	this->state.mining_enabled = false;
	this->state.offload_enabled = false;
	this->state.mining_complete = false;
	this->state.mining_lowered_hopper = false;
}

void Robot::TeleopPeriodic()
{
	this->state.serial_enabled = false;

	if(this->state.mining_enabled || this->state.offload_enabled) return;

	this->periodic_handle_teleop_input();
}


void Robot::DisabledInit()
{
	this->disable_motors();
	this->state.serial_enabled = false;
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
