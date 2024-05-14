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
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/AnalogPotentiometer.h>
#include "frc/Joystick.h"
#include "frc/XboxController.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "ctre/Phoenix.h"

#include "LogitechConstants.hpp"
#include "SenderNT.hpp"

#include <wpi/sendable/SendableBuilder.h>

#include "wpimath/MathShared.h"


using namespace ctre::phoenix6;

typedef ctre::phoenix6::hardware::TalonFX TalonFX6;

class Robot : public frc::TimedRobot, public wpi::Sendable {
public:
	Robot();
	~Robot();

public:
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

protected:
	void InitSendable(wpi::SendableBuilder& builder) override;

	void configure_motors();
	void disable_motors();
	void stop_all();

protected:
	void mining_init();
	void mining_shutdown();
	void offload_init();
	void offload_shutdown();

protected:
	void periodic_handle_serial_control();
	void periodic_handle_mining();
	void periodic_handle_offload();
	void periodic_handle_teleop_input();


private:
	// state
	class State : public wpi::Sendable{
	public:
		enum class MiningStage {
			INITIALIZING = 0,
			LOWERING_HOPPER = 1,
			TRAVERSING = 2,
			RAISING_HOPPER = 3,
			FINISHED = 4
		};
		enum class OffloadingStage {
			INITIALIZING = 0,
			BACKING_UP = 1,
			RAISING_HOPPER = 2,
			OFFLOADING = 3,
			LOWERING_HOPPER = 4,
			FINISHED = 5
		};

		double driving_speed_scalar = Robot::DRIVING_MEDIUM_SPEED_SCALAR;

		struct {
			bool enabled = false;
			MiningStage stage = MiningStage::FINISHED;

			std::chrono::system_clock::time_point traversal_start_time;

			double target_mining_time = Robot::MINING_RUN_TIME_SECONDS;

		} mining;
		struct {
			bool enabled = false;
			OffloadingStage stage = OffloadingStage::FINISHED;

			std::chrono::system_clock::time_point start_time, dump_start_time;

			double
				tele_target_backup_time = Robot::TELE_OFFLOAD_BACKUP_TIME_SECONDS,
				auto_target_backup_time = Robot::AUTO_OFFLOAD_BACKUP_TIME_SECONDS,
				target_dump_time = Robot::OFFLOAD_DUMP_TIME;

		} offload;

	public:
		void reset_auto_states();

		void InitSendable(wpi::SendableBuilder& builder) override;

	};
	State state;

	struct {
		frc::SerialPort
			port = frc::SerialPort{
				115200,
				frc::SerialPort::Port::kOnboard,
				8,
				frc::SerialPort::Parity::kParity_None,
				frc::SerialPort::StopBits::kStopBits_One
			};
		uint16_t
			input_i{ 0 };
		char
			input_buffer[32];
			
		bool enabled = false;

	} serial;

	frc::Joystick
		logitech{ 0 };
	frc::AnalogPotentiometer
		hopper_actuator_pot{ 0 };
	TalonFX6
		track_right{ 0 },
		track_left{ 1 },
		trencher{ 2 },
		hopper_belt{ 3 };
	WPI_TalonSRX
		hopper_actuator{ 4 };

	TalonFX6* motors[2] = {
		&track_right,
		&track_left,
	};

	// SenderNT telemetry_sender;

	static constexpr auto
	// motor physical speed targets
		TRENCHER_MAX_VELO = 80_tps,
		TRENCHER_MINING_VELO = 80_tps,
		HOPPER_BELT_MAX_VELO = 45_tps,
		HOPPER_BELT_MAX_MINING_VELO = 10_tps,
		TRACKS_MAX_VELO = 125_tps,
		TRACKS_MINING_VELO = 8_tps,
		TRACKS_OFFLOAD_VELO = TRACKS_MAX_VELO * 0.25;

	static constexpr auto
		MOTOR_SETPOINT_ACC = 5_tr_per_s_sq;

	static constexpr double
	// motor constants
		GENERIC_MOTOR_kP = 0.11,	// An error of 1 rotation per second results in 2V output
		GENERIC_MOTOR_kI = 0.5,		// An error of 1 rotation per second increases output by 0.5V every second
		GENERIC_MOTOR_kD = 0.0001,	// A change of 1 rotation per second squared results in 0.0001 volts output
		GENERIC_MOTOR_kV = 0.12,	// Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
	// driving
		DRIVING_DEADZONE_SCALAR = 0.1,
		DRIVING_LOW_SPEED_SCALAR = 0.3,
		DRIVING_MEDIUM_SPEED_SCALAR = 0.7,
		DRIVING_HIGH_SPEED_SCALAR = 1.0,
	// hopper
		HOPPER_ACTUATOR_PLUNGE_SPEED = 0.65,
		HOPPER_ACTUATOR_EXTRACT_SPEED = 0.80,
		HOPPER_ACUTATOR_MOVE_SPEED = 1.0,	// all other movement (ie. dumping)
	// actuator potentiometer target values
		OFFLOAD_POT_VALUE = 0.95,
		TRAVERSAL_POT_VALUE = 0.50,
		AUTO_TRANSPORT_POT_VALUE = 0.45,
		MINING_POT_VALUE = 0.03;

	static constexpr double
	// timed operations
		MINING_RUN_TIME_SECONDS = 15.0,           // teleauto mining run time
		TELE_OFFLOAD_BACKUP_TIME_SECONDS = 3.0,   // teleauto offload duration
		AUTO_OFFLOAD_BACKUP_TIME_SECONDS = 1.0,
		OFFLOAD_DUMP_TIME = 6.0;
	// auto belt duty cycle
	static constexpr double
		HOPPER_BELT_TIME_ON_SECONDS = 1.0,
		HOPPER_BELT_TIME_OFF_SECONDS = 2.5;

	static constexpr int
		DISABLE_ALL_ACTIONS_BUTTON_IDX = LogitechConstants::BUTTON_A,

		TELEOP_LOW_SPEED_BUTTON_IDX = LogitechConstants::BUTTON_B,
		TELEOP_MEDIUM_SPEED_BUTTON_IDX = LogitechConstants::BUTTON_Y,
		TELEOP_HIGH_SPEED_BUTTON_IDX = LogitechConstants::BUTTON_X,

		TELEOP_DRIVE_X_AXIS_IDX = LogitechConstants::LEFT_JOY_X,
		TELEOP_DRIVE_Y_AXIS_IDX = LogitechConstants::LEFT_JOY_Y,

		TELEOP_TRENCHER_SPEED_AXIS_IDX = LogitechConstants::RIGHT_TRIGGER,
		TELEOP_TRENCHER_INVERT_BUTTON_IDX = LogitechConstants::RB,

		TELEOP_HOPPER_SPEED_AXIS_IDX = LogitechConstants::LEFT_TRIGGER,
		TELEOP_HOPPER_INVERT_BUTTON_IDX = LogitechConstants::LB,
		TELEOP_HOPPER_ACTUATE_AXIS_IDX = LogitechConstants::RIGHT_JOY_Y,

		TELEAUTO_MINING_INIT_POV = LogitechConstants::DPAD_UP_POV, /*Mining Init Is Up*/
		TELEAUTO_MINING_STOP_POV = LogitechConstants::DPAD_DOWN_POV, /*Mining Stop Is Down*/
		TELEAUTO_OFFLOAD_INIT_POV = LogitechConstants::DPAD_RIGHT_POV,  /*Offload Init Is Right*/
		TELEAUTO_OFFLOAD_STOP_POV = LogitechConstants::DPAD_LEFT_POV;  /*Offload Stop Is Left*/
	

};
