// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogPotentiometer.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>

#include <chrono>
#include <iostream>
#include <ctime>
#include <cmath>
#include <vector>
#include <deque>
#include <numeric>


using system_time = std::chrono::system_clock;
using system_time_point = system_time::time_point;

static constexpr long double
	PI = 3.14159265358979323846;
static constexpr char
	XON = 0x13,
	XOFF = 0x11;

namespace util {

	inline double seconds_since(const system_time_point& tp)
	{
		return std::chrono::duration<double>{ system_time::now() - tp }.count();
	}

	frc::DifferentialDrive::WheelSpeeds computeWheelScalars(double x, double y, double deadzone)
	{
		const double augmented_angle = std::atan2(x, y) + (PI / 4.0);	// x and y are inverted to make a CW "heading" angle
		double magnitude = std::sqrt(x*x + y*y);
		if (magnitude < 0.1) return { 0.0, 0.0 };

		return {
			magnitude * std::sin(augmented_angle),
			magnitude * std::cos(augmented_angle)		// this is the same as cos("raw theta" - pi/4) like from the original code
		};
	}

	void add_fx6_dbg_info(wpi::SendableBuilder& builder, TalonFX6& motor, std::string prefix)
	{
		builder.AddDoubleProperty(prefix + "/duty_cycle", [&] { return motor.Get(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/voltage", [&] { return motor.GetMotorVoltage().GetValueAsDouble(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/position", [&] { return motor.GetPosition().GetValueAsDouble(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/current", [&] { return motor.GetStatorCurrent().GetValueAsDouble(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/temp", [&] { return motor.GetDeviceTemp().GetValueAsDouble(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/velocity", [&] { return motor.GetVelocity().GetValueAsDouble(); }, nullptr);
	}
	void add_srx_dbg_info(wpi::SendableBuilder& builder, WPI_TalonSRX& motor, std::string prefix)
	{
		builder.AddDoubleProperty(prefix + "/duty_cycle", [&] { return motor.GetMotorOutputPercent(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/voltage", [&] { return motor.GetMotorOutputVoltage(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/position", [&] { return motor.GetSelectedSensorPosition(0 /*primary closed loop idx*/); }, nullptr);
		builder.AddDoubleProperty(prefix + "/current", [&] { return motor.GetStatorCurrent(); }, nullptr);
		builder.AddDoubleProperty(prefix + "/temp", [&] { return motor.GetTemperature(); }, nullptr);
		// builder.AddDoubleProperty(prefix + "/velocity", [&] { return motor.GetSelectedSensorVelocity(0 /*primary closed loop idx*/); }, nullptr);
	}

}


enum class SerialCommand : int32_t
{
	INVALID = -1,
	MANUAL_TRACKS = 0,
	START_MINING = 1,
	STOP_MINING = 2,
	START_OFFLOAD = 3,
	STOP_OFFLOAD = 4,
	TOTAL = 5
};



void Robot::State::reset_auto_states()
{
	this->mining.enabled = false;
	this->offload.enabled = false;
	this->mining.stage = Robot::State::MiningStage::FINISHED;
	this->offload.stage = Robot::State::OffloadingStage::FINISHED;
}

void Robot::State::InitSendable(wpi::SendableBuilder& builder)
{
	builder.AddBooleanProperty("state/mining_enabled", [this](){return this->mining.enabled;}, nullptr);
	builder.AddBooleanProperty("state/offload_enabled", [this](){return this->offload.enabled;}, nullptr);
	builder.AddIntegerProperty("state/mining_stage", [this](){return static_cast<int>(this->mining.stage);}, nullptr);
	builder.AddIntegerProperty("state/offload_stage", [this](){return static_cast<int>(this->offload.stage);}, nullptr);
	builder.AddIntegerProperty("state/mining_serial_control_state", [this](){return static_cast<int>(this->mining.serial_control);}, nullptr);
	builder.AddIntegerProperty("state/offload_serial_control_state", [this](){return static_cast<int>(this->offload.serial_control);}, nullptr);
	builder.AddDoubleProperty("state/driving_speed_scalar", [this](){return this->driving_speed_scalar;}, nullptr);

	builder.AddDoubleProperty("tuning/mining_runtime", [this](){return this->mining.target_mining_time;}, [this](double v){this->mining.target_mining_time = v;});
	builder.AddDoubleProperty("tuning/tele_offload_backup_time", [this](){return this->offload.tele_target_backup_time;}, [this](double v){this->offload.tele_target_backup_time = v;});
	builder.AddDoubleProperty("tuning/auto_offload_backup_time", [this](){return this->offload.auto_target_backup_time;}, [this](double v){this->offload.auto_target_backup_time = v;});
	builder.AddDoubleProperty("tuning/offload_dump_time", [this](){return this->offload.target_dump_time;}, [this](double v){this->offload.target_dump_time = v;});
};



Robot::Robot() /*: telemetry_sender{ "Robot" }*/
{
	this->serial.port.SetTimeout(units::second_t{ 0.1 });
}
Robot::~Robot() {}


// ------------ TELEMETRY ---------------

void Robot::InitSendable(wpi::SendableBuilder& builder)
{
	util::add_fx6_dbg_info(builder, track_right, "TrackRight");
	util::add_fx6_dbg_info(builder, track_left, "TrackLeft");
	util::add_fx6_dbg_info(builder, trencher, "Trencher");
	util::add_fx6_dbg_info(builder, hopper_belt, "HopperBelt");
	util::add_srx_dbg_info(builder, hopper_actuator, "HopperActuator");
	builder.AddDoubleProperty("HopperActuator/potentiometer", [this](){ return this->hopper_actuator_pot.Get(); }, nullptr);
	builder.AddBooleanProperty("state/serial_enabled", [this](){return this->serial.enabled;}, nullptr);

	this->state.InitSendable(builder);
}


// --------- Setup/Helpers ----------

void Robot::configure_motors()
{
	configs::TalonFXConfiguration generic_config{}/*, tracks_config{}*/;

	/* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
	generic_config.Slot0.kP = Robot::GENERIC_MOTOR_kP;
	generic_config.Slot0.kI = Robot::GENERIC_MOTOR_kI;
	generic_config.Slot0.kD = Robot::GENERIC_MOTOR_kD;
	generic_config.Slot0.kV = Robot::GENERIC_MOTOR_kV;

	// tracks_config.CurrentLimits.StatorCurrentLimitEnable = false;
	generic_config.CurrentLimits.StatorCurrentLimitEnable = false;

	this->track_left.GetConfigurator().Apply(generic_config);
	this->track_right.GetConfigurator().Apply(generic_config);

	this->hopper_belt.GetConfigurator().Apply(generic_config);
	this->trencher.GetConfigurator().Apply(generic_config);

	this->hopper_actuator.EnableCurrentLimit(false);

	this->trencher.SetInverted(true);
	this->hopper_belt.SetInverted(false);
	this->track_right.SetInverted(true);
}

void Robot::disable_motors()
{
	this->track_right.Set(0);
	this->track_left.Set(0);
	this->trencher.Set(0);
	this->hopper_belt.Set(0);
	this->hopper_actuator.Set(0);
}

void Robot::stop_all()
{
	this->state.reset_auto_states();
	this->disable_motors();
}

void Robot::disable_serial()
{
	this->serial.enabled = false;
	this->state.mining.serial_control = Robot::State::SerialControlState::DISABLED;
	this->state.offload.serial_control = Robot::State::SerialControlState::DISABLED;
}

void Robot::send_serial_success()
{
	static char result = 0;
	this->serial.port.Write(&result, sizeof(result));
	this->serial.port.Write(&XOFF, sizeof(XOFF)); // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
}



// ------------- Mining/Offload init and shutdown --------------

void Robot::mining_init(bool serial)
{
	if(!this->state.mining.enabled && !this->state.offload.enabled)
	{
		this->stop_all();

		this->state.mining.enabled = true;
		this->state.mining.stage = Robot::State::MiningStage::INITIALIZING;
		this->state.mining.serial_control =
			(this->serial.enabled && serial) ?
				this->state.mining.serial_control = Robot::State::SerialControlState::STARTED :
				this->state.mining.serial_control = Robot::State::SerialControlState::DISABLED;
	}
}

void Robot::mining_shutdown(bool serial)
{
	if (this->state.mining.enabled && !this->state.offload.enabled)
	{
		if(this->serial.enabled && serial)
		{
			this->state.mining.serial_control = Robot::State::SerialControlState::CANCELLED;
			return;
		}
		else this->state.mining.serial_control = Robot::State::SerialControlState::DISABLED;

		this->stop_all();

		this->state.mining.enabled = false;
		this->state.mining.stage = Robot::State::MiningStage::FINISHED;
	}
}

void Robot::offload_init(bool serial)
{
	if(!this->state.offload.enabled && !this->state.mining.enabled)
	{
		this->stop_all();

		this->state.offload.enabled = true;
		this->state.offload.stage = Robot::State::OffloadingStage::INITIALIZING;
		this->state.offload.serial_control =
			(this->serial.enabled && serial) ?
				this->state.offload.serial_control = Robot::State::SerialControlState::STARTED :
				this->state.offload.serial_control = Robot::State::SerialControlState::DISABLED;
	}
}

void Robot::offload_shutdown(bool serial)
{
	if(this->state.offload.enabled && !this->state.mining.enabled)
	{
		if(this->serial.enabled && serial)
		{
			this->state.offload.serial_control = Robot::State::SerialControlState::CANCELLED;
			return;
		}
		else this->state.offload.serial_control = Robot::State::SerialControlState::DISABLED;

		this->stop_all();

		this->state.offload.enabled = false;
		this->state.offload.stage = Robot::State::OffloadingStage::FINISHED;
	}
}





// ----------------- Periodic Handlers ----------------------

void Robot::periodic_handle_serial_control()
{
	if(this->serial.enabled)	// basically if in autonomous mode
	{
		if(this->IsSimulation())	// manually call the serial commands
		{
			const bool
				any_ops_running = (this->state.mining.enabled || this->state.offload.enabled),
				definite_is_mining = (this->state.mining.enabled && !this->state.offload.enabled),
				definite_is_offload = (!this->state.mining.enabled && this->state.offload.enabled);

			if(!any_ops_running && logitech.GetPOV(0) == Robot::TELEAUTO_MINING_INIT_POV) {		// dpad top
				this->mining_init(true);
			}
			if(definite_is_mining && logitech.GetPOV(0) == Robot::TELEAUTO_MINING_STOP_POV) {	// dpad bottom
				this->mining_shutdown(true);
			}
			if(!any_ops_running && logitech.GetPOV(0) == Robot::TELEAUTO_OFFLOAD_INIT_POV) {	// dpad right
				this->offload_init(true);
			}
			if(definite_is_offload && logitech.GetPOV(0) == Robot::TELEAUTO_OFFLOAD_STOP_POV) {	// dpad left
				this->offload_shutdown(true);
			}
		}	// end testing code
		else
		if(this->serial.port.Read(this->serial.input_buffer, 1) == 1 && this->serial.input_buffer[0] == XON) //reads 1 byte, if its xon it continues, clear any incomplete buffer and listens to handshake
		{
			this->serial.port.Write(&XON, sizeof(XON)); // response to being on
			if(this->serial.port.Read(this->serial.input_buffer, 4)) // in corresponding to opcode 4 byte int
			{
				SerialCommand cmd = SerialCommand::INVALID;
				const int32_t func_val = *reinterpret_cast<int32_t*>(this->serial.input_buffer);	// first 4 bytes are the function number
				if(func_val >= 0 && func_val < static_cast<int32_t>(SerialCommand::TOTAL))
					cmd = static_cast<SerialCommand>(func_val);

				switch(cmd) // choose what to do based on opcode
				{
					case SerialCommand::MANUAL_TRACKS:
					{
						if(this->state.mining.enabled || this->state.offload.enabled) break;

						this->serial.port.Read(this->serial.input_buffer, 12);	// read the rest of the message (4 bytes for motor num and 8 bytes for velocity)

						int32_t motor_number = *reinterpret_cast<int32_t*>(this->serial.input_buffer);
						if(motor_number < 0 || motor_number > 1) break;

						double tps_value = *reinterpret_cast<double*>(this->serial.input_buffer + 4);
						motors[motor_number]->SetControl(
							ctre::phoenix6::controls::VelocityVoltage{
								units::angular_velocity::turns_per_second_t{ tps_value },
								Robot::MOTOR_SETPOINT_ACC,
								false
							}
						);

						// this->send_serial_success();
						break;
					}
					case SerialCommand::START_MINING:
					{
						this->mining_init(true);
						// this->send_serial_success();
						break;
					}
					case SerialCommand::STOP_MINING:
					{
						this->mining_shutdown(true);
						break;
					}
					case SerialCommand::START_OFFLOAD:
					{
						this->offload_init(true);
						break;
					}
					case SerialCommand::STOP_OFFLOAD:
					{
						this->offload_shutdown(true);
						// this->send_serial_success();
						break;
					}
					case SerialCommand::INVALID:
					case SerialCommand::TOTAL:
					default:
					{}
				}
				this->send_serial_success();
			}
		}
	}
}

void Robot::periodic_handle_mining()
{
	if(this->state.mining.enabled) {
		switch(this->state.mining.stage) {
			case Robot::State::MiningStage::INITIALIZING:
			{
				this->state.mining.stage = Robot::State::MiningStage::LOWERING_HOPPER;
				// fallthrough to process the next stage
			}
			case Robot::State::MiningStage::LOWERING_HOPPER:
			{
				const double pot_val = this->hopper_actuator_pot.Get();
				if( pot_val > Robot::MINING_POT_VALUE && (!this->serial.enabled ||
					this->state.mining.serial_control == Robot::State::SerialControlState::STARTED))
				{
					// set trencher
					this->trencher.SetControl(
						ctre::phoenix6::controls::VelocityVoltage{
							Robot::TRENCHER_MINING_VELO,
							Robot::MOTOR_SETPOINT_ACC,
							false
						}
					);
					// set actuator
					if(pot_val > Robot::TRAVERSAL_POT_VALUE)
						this->hopper_actuator.Set(Robot::HOPPER_ACUTATOR_MOVE_SPEED);
					else
						this->hopper_actuator.Set(Robot::HOPPER_ACTUATOR_PLUNGE_SPEED);

					break;
				}
				else
				{
					this->hopper_actuator.Set(0);
					// if(this->state.mining.serial_control != Robot::State::SerialControlState::DISABLED) this->send_serial_success();
					this->state.mining.traversal_start_time = system_time::now();
					this->state.mining.stage = Robot::State::MiningStage::TRAVERSING;
					// allow fallthrough bc we might as well start processing traversal
				}
			}
			case Robot::State::MiningStage::TRAVERSING:
			{
				if( (this->state.mining.serial_control == Robot::State::SerialControlState::STARTED) ||
					(!this->serial.enabled && util::seconds_since(this->state.mining.traversal_start_time) < this->state.mining.target_mining_time))
				{
					// set trencher
					this->trencher.SetControl(
						ctre::phoenix6::controls::VelocityVoltage{
							Robot::TRENCHER_MINING_VELO,
							Robot::MOTOR_SETPOINT_ACC,
							false
						}
					);

					// handle hopper duty cycle
					const bool run_hopper = std::abs(std::fmod(util::seconds_since(state.offload.start_time), Robot::HOPPER_BELT_TIME_ON_SECONDS + Robot::HOPPER_BELT_TIME_OFF_SECONDS)) < Robot::HOPPER_BELT_TIME_ON_SECONDS;
					if(run_hopper)
					{
						ctre::phoenix6::controls::VelocityVoltage
						vel_command{
							-Robot::HOPPER_BELT_MAX_MINING_VELO,
							Robot::MOTOR_SETPOINT_ACC,
							false
						};
						this->hopper_belt.SetControl(vel_command);
					}
					else this->hopper_belt.Set(0);

					// set tracks
					ctre::phoenix6::controls::VelocityVoltage
						vel_command{
							Robot::TRACKS_MINING_VELO,
							Robot::MOTOR_SETPOINT_ACC,
							false
						};
					this->track_left.SetControl(vel_command);
					this->track_right.SetControl(vel_command);

					break;
				}
				else
				{
					this->track_left.Set(0);
					this->track_right.Set(0);
					this->state.mining.stage = Robot::State::MiningStage::RAISING_HOPPER;
					// allow fallthrough
				}
			}
			case Robot::State::MiningStage::RAISING_HOPPER:
			{
				const double pot_val = this->hopper_actuator_pot.Get();
				if(pot_val < Robot::AUTO_TRANSPORT_POT_VALUE)
				{
					// set trencher
					this->trencher.SetControl(
						ctre::phoenix6::controls::VelocityVoltage{
							Robot::TRENCHER_MINING_VELO,
							Robot::MOTOR_SETPOINT_ACC,
							false
						}
					);
					// set actuator
					this->hopper_actuator.Set(-Robot::HOPPER_ACTUATOR_EXTRACT_SPEED);
					break;
				}
				else
				{
					this->hopper_actuator.Set(0);
					// if(this->state.mining.serial_control == Robot::State::SerialControlState::CANCELLED) this->send_serial_success();
					this->state.mining.stage = Robot::State::MiningStage::FINISHED;
					// fallthrough to call shutdown
				}
			}
			case Robot::State::MiningStage::FINISHED:
			{
				this->mining_shutdown();
			}
			default:
			{
				// nothing
			}
		}
	}
}

void Robot::periodic_handle_offload()
{
	if(this->state.offload.enabled) {
		switch(this->state.offload.stage) {
			case Robot::State::OffloadingStage::INITIALIZING:
			{
				this->state.offload.start_time = system_time::now();
				this->state.offload.stage = Robot::State::OffloadingStage::BACKING_UP;
				// fallthrough
			}
			case Robot::State::OffloadingStage::BACKING_UP:
			{
				const double duration = util::seconds_since(this->state.offload.start_time);
				if( (this->serial.enabled && duration < this->state.offload.auto_target_backup_time) ||
					(!this->serial.enabled && duration < this->state.offload.tele_target_backup_time) )		// use serial_control state for this if deemed reliable enough
				{
					// drive backwards
					ctre::phoenix6::controls::VelocityVoltage
						vel_command{
							-Robot::TRACKS_OFFLOAD_VELO,
							Robot::MOTOR_SETPOINT_ACC,
							false
						};
					track_left.SetControl(vel_command);
					track_right.SetControl(vel_command);

					break;
				}
				else
				{
					track_left.Set(0);
					track_right.Set(0);

					this->state.offload.stage = Robot::State::OffloadingStage::RAISING_HOPPER;
					// fallthrough and process the next stage
				}
			}
			case Robot::State::OffloadingStage::RAISING_HOPPER:
			{
				if(this->hopper_actuator_pot.Get() < Robot::OFFLOAD_POT_VALUE)
				{
					this->hopper_actuator.Set(-Robot::HOPPER_ACUTATOR_MOVE_SPEED);	// dump
					break;
				}
				else
				{
					this->hopper_actuator.Set(0);
					this->state.offload.dump_start_time = system_time::now();
					this->state.offload.stage = Robot::State::OffloadingStage::OFFLOADING;
					// fallthrough
				}
			}
			case Robot::State::OffloadingStage::OFFLOADING:
			{
				if(util::seconds_since(this->state.offload.dump_start_time) < this->state.offload.target_dump_time)
				{
					// set hopper belt
					this->hopper_belt.SetControl(
						ctre::phoenix6::controls::VelocityVoltage{
							-Robot::HOPPER_BELT_MAX_VELO,
							Robot::MOTOR_SETPOINT_ACC,
							false
						}
					);
					break;
				}
				else
				{
					this->hopper_belt.Set(0);
					this->state.offload.stage = Robot::State::OffloadingStage::LOWERING_HOPPER;
					// fallthrough
				}
			}
			case Robot::State::OffloadingStage::LOWERING_HOPPER:
			{
				if(this->hopper_actuator_pot.Get() > Robot::TRAVERSAL_POT_VALUE)
				{
					this->hopper_actuator.Set(Robot::HOPPER_ACUTATOR_MOVE_SPEED);
					break;
				}
				else
				{
					this->hopper_actuator.Set(0);
					// if(this->state.offload.serial_control != Robot::State::SerialControlState::DISABLED) this->send_serial_success();
					this->state.offload.stage = Robot::State::OffloadingStage::FINISHED;
					// fallthrough
				}
			}
			case Robot::State::OffloadingStage::FINISHED:
			{
				this->offload_shutdown();
			}
			default:
			{
				// nothing
			}
		}
	}
}



void Robot::periodic_handle_teleop_input()
{
	// ------------ HARD RESET ------------
	if(logitech.GetRawButtonPressed(Robot::DISABLE_ALL_ACTIONS_BUTTON_IDX))
	{
		this->stop_all();
		this->disable_serial();	// resets internal serial_control states
	}

	const bool
		any_ops_running = (this->state.mining.enabled || this->state.offload.enabled),
		definite_is_mining = (this->state.mining.enabled && !this->state.offload.enabled),
		definite_is_offload = (!this->state.mining.enabled && this->state.offload.enabled);

	// ---------- TELEAUTO CONTROl ----------
	{
		if(!any_ops_running && logitech.GetPOV(0) == Robot::TELEAUTO_MINING_INIT_POV) {		// dpad top
			this->mining_init(false);
		}
		if(definite_is_mining && logitech.GetPOV(0) == Robot::TELEAUTO_MINING_STOP_POV) {	// dpad bottom
			this->mining_shutdown(false);
		}
		if(!any_ops_running && logitech.GetPOV(0) == Robot::TELEAUTO_OFFLOAD_INIT_POV) {	// dpad right
			this->offload_init(false);
		}
		if(definite_is_offload && logitech.GetPOV(0) == Robot::TELEAUTO_OFFLOAD_STOP_POV) {	// dpad left
			this->offload_shutdown(false);
		}
	}

	if(this->state.mining.enabled || this->state.offload.enabled) return;

	// ------- DRIVE CONTROL ------------
	{
		// handle driving speed updates
		if(logitech.GetRawButtonPressed(Robot::TELEOP_LOW_SPEED_BUTTON_IDX)) this->state.driving_speed_scalar = Robot::DRIVING_LOW_SPEED_SCALAR;
		if(logitech.GetRawButtonPressed(Robot::TELEOP_MEDIUM_SPEED_BUTTON_IDX)) this->state.driving_speed_scalar = Robot::DRIVING_MEDIUM_SPEED_SCALAR;
		if(logitech.GetRawButtonPressed(Robot::TELEOP_HIGH_SPEED_BUTTON_IDX)) this->state.driving_speed_scalar = Robot::DRIVING_HIGH_SPEED_SCALAR;

		ctre::phoenix6::controls::VelocityVoltage
			vel_command{ 0_tps, Robot::MOTOR_SETPOINT_ACC, false };
		const double
			stick_x = logitech.GetRawAxis(Robot::TELEOP_DRIVE_X_AXIS_IDX),
			stick_y = -logitech.GetRawAxis(Robot::TELEOP_DRIVE_Y_AXIS_IDX);	// forward y is positive

		auto track_speeds = util::computeWheelScalars(stick_x, stick_y, Robot::DRIVING_DEADZONE_SCALAR);

		// set drive velocities
		track_right.SetControl(
			vel_command.WithVelocity(
				Robot::TRACKS_MAX_VELO * this->state.driving_speed_scalar * track_speeds.right
			));
		track_left.SetControl(
			vel_command.WithVelocity(
				Robot::TRACKS_MAX_VELO * this->state.driving_speed_scalar * track_speeds.left
			));
	}
	// -------- TRENCHER CONTROL -------------
	{
		double trencher_speed = logitech.GetRawAxis(Robot::TELEOP_TRENCHER_SPEED_AXIS_IDX);
		if (logitech.GetRawButton(Robot::TELEOP_TRENCHER_INVERT_BUTTON_IDX)) trencher_speed *= -1.0;

		// set trencher velocity
		trencher.SetControl(
			ctre::phoenix6::controls::VelocityVoltage{
				(Robot::TRENCHER_MAX_VELO * trencher_speed),
				Robot::MOTOR_SETPOINT_ACC,
				false
			}
		);
	}
	// -------- HOPPER CONTROL -------------
	{
		double hopper_belt_speed = -logitech.GetRawAxis(Robot::TELEOP_HOPPER_SPEED_AXIS_IDX);
		if (logitech.GetRawButton(Robot::TELEOP_HOPPER_INVERT_BUTTON_IDX)) hopper_belt_speed *= -1.0;

		// set hopper belt
		hopper_belt.SetControl(
			ctre::phoenix6::controls::VelocityVoltage{
				(Robot::HOPPER_BELT_MAX_VELO * hopper_belt_speed),
				Robot::MOTOR_SETPOINT_ACC,
				false
			}
		);
		// set actutor power
		hopper_actuator.Set(
			logitech.GetRawAxis(Robot::TELEOP_HOPPER_ACTUATE_AXIS_IDX)
		);
	}
}





// -------------- TimedRobot Overrides --------------

void Robot::RobotInit()
{
	// setup motors
	this->configure_motors();
	this->disable_motors();
	this->disable_serial();

	// this->telemetry_sender.putData(this);
	frc::SmartDashboard::PutData("robot", this);

	if(this->IsReal()) {
		frc::DataLogManager::Start();	// setup offline logging
		frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());	// log driverstation inputs
	}
}

void Robot::RobotPeriodic()
{
	// putting this here for now
	// this->periodic_handle_serial_control();

	// run periodic control
	this->periodic_handle_mining();
	this->periodic_handle_offload();

	// this->telemetry_sender.updateValues();
}


void Robot::AutonomousInit()
{
	this->stop_all();
	this->serial.enabled = true;
}

void Robot::AutonomousPeriodic() {}


void Robot::TeleopInit()
{
	this->stop_all();
	this->disable_serial();
}

void Robot::TeleopPeriodic()
{
	this->periodic_handle_teleop_input();
}


void Robot::DisabledInit()
{
	this->stop_all();
	this->disable_serial();
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
