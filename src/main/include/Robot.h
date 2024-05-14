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
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
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
typedef WPI_TalonFX TalonFX5;

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
		bool
			mining_enabled = false,
			offload_enabled = false,
			teleauto_operation_complete = false,
			hopper_enabled = false,
			offload_traversal_reached = false,
			mining_lowered_hopper = false;

		double
			driving_speed_scalar = Robot::DRIVING_MEDIUM_SPEED_SCALAR,
			teleauto_mining_runtime = Robot::MINING_RUN_TIME_SECONDS,
			teleauto_offload_backup_time = Robot::TELE_OFFLOAD_BACKUP_TIME_SECONDS,
			teleauto_offload_dump_time = Robot::OFFLOAD_DUMP_TIME;

		std::chrono::system_clock::time_point
			auto_operation_start_time,
			offload_traversal_start_time;

		void reset();

		void InitSendable(wpi::SendableBuilder& builder) override;

	};
	State state;

	frc::Joystick
		logitech{ 0 };
	frc::AnalogPotentiometer
		hopper_actuator_pot{ 0 };
	TalonFX6
		track_right{ 0 },
		track_left{ 1 },
		trencher{ 2 },
		hopper_belt{ 3 };
	TalonFX5
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

	static constexpr double
	// driving
		DRIVING_DEADZONE_SCALAR = 0.1,
		DRIVING_LOW_SPEED_SCALAR = 0.3,
		DRIVING_MEDIUM_SPEED_SCALAR = 0.7,
		DRIVING_HIGH_SPEED_SCALAR = 1.0,
	// hopper
		HOPPER_ACTUATOR_MAX_PERCENT = 0.2,
	// actuator potentiometer target values
		OFFLOAD_POT_VALUE = 0.95,
		TRAVERSAL_POT_VALUE = 0.5,
		AUTO_TRANSPORT_POT_VALUE = 0.45,
		MINING_POT_VALUE = 0.03,
	// component speeds during operation
		MINING_HOPPER_MOVE_PERCENT = 0.75,
		OFFLOAD_HOPPER_MOVE_PERCENT = 1.0;

	static constexpr double
	// timed operations
		MINING_RUN_TIME_SECONDS = 10.0,           // teleauto mining run time
		TELE_OFFLOAD_BACKUP_TIME_SECONDS = 1.5,   // teleauto offload duration
		AUTO_OFFLOAD_BACKUP_TIME_SECONDS = 1.0,
		OFFLOAD_DUMP_TIME = 6.0;

	// // constants for timing of hopper movement during mining
	// static constexpr int
	// 	hopper_belt_mine_wait_time = 750,	// in milliseconds
	// 	hopper_belt_mine_run_time = 100;	// in milliseconds

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

		TELEAUTO_MINING_INIT_POV = 0, /*Mining Init Is Up*/
		TELEAUTO_MINING_STOP_POV = 180, /*Mining Stop Is Down*/
		TELEAUTO_OFFLOAD_INIT_POV = 90,  /*Offload Init Is Right*/
		TELEAUTO_OFFLOAD_STOP_POV = 270;  /*Offload Stop Is Left*/

};
