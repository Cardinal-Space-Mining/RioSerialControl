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

#define BUF_SIZE 13
#define MOTOR_COUNT 5

constexpr int PIGEON_CAN_ID = 5; // << Value!!!

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
	void InitSendable(wpi::SendableBuilder& builder) override;

protected:
	uint8_t get_moving_avg();

	void configure_motors();
    void disable_motors();

    uint8_t mining_init();
    uint8_t mining_shutdown();
    uint8_t offload_init();
    uint8_t offload_shutdown();

	void periodic_handle_serial_control();
	void periodic_handle_mining();
	void periodic_handle_offload();
	void periodic_handle_teleop_input();

	// void InitSendable(wpi::SendableBuilder &) override; // use for loggin motor data

private:
	// frc::SerialPort serial = frc::SerialPort(9600);
	frc::SerialPort serial = frc::SerialPort(115200, frc::SerialPort::Port::kOnboard, 8, frc::SerialPort::Parity::kParity_None, frc::SerialPort::StopBits::kStopBits_One);

	// char * input_buffer = (char *)malloc(sizeof(char) * BUF_SIZE);
	// uint16_t input_i = 0;
	// char * xon = (char *)malloc(sizeof(char) * 1);
	// char * xoff = (char *)malloc(sizeof(char) * 1);
	// char * time_buffer = (char *)malloc(sizeof(char) * 13);
	uint16_t input_i{ 0 };
	char
		input_buffer[BUF_SIZE],
		xon[1],
		xoff[1],
		time_buffer[13];

	// state
	class State : public wpi::Sendable{
	public:
		bool
			serial_enabled = false,
			mining_enabled = false,
			offload_enabled = false,
			mining_complete = false,
			hopper_enabled = false,
			offload_traversal_reached = false,
			mining_started_trencher = false;

		double
			driving_speed_scalar = Robot::DRIVING_MEDIUM_SPEED_SCALAR;

		std::chrono::system_clock::time_point
			auto_operation_start_time,
			offload_traversal_start_time;

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
		hopper_belt{ 3 },
		hopper_actuator{ 4 };

	TalonFX6* motors[2] = {
		&track_right,
		&track_left,
	};

	SenderNT telemetry_sender;

	// Variables for moving average
	uint8_t movingAvgRange = 10;
	double trenchAvgCurrent = 0.0;
	static constexpr double avg_current_thresh = 50.0;
	std::deque<double> motorDataList;

	static constexpr auto
	// motor physical speed targets
		TRENCHER_MAX_VELO = 80_tps,
		HOPPER_BELT_MAX_VELO = 30_tps,
		HOPPER_BELT_MAX_MINING_VELO = 10_tps,
		TRACKS_MAX_VELO = 125_tps,
		TRACKS_MINING_MAX_VELO = 8_tps;

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
		MINING_POT_VALUE = 0.03;

	static constexpr int
	// timed operations
		MINING_RUN_TIME_SECONDS = 10,           // teleauto mining run time
		TELE_OFFLOAD_BACKUP_TIME_SECONDS = 3,   // teleauto offload duration
		AUTO_OFFLOAD_BACKUP_TIME_SECONDS = 1,
		OFFLOAD_TOTAL_RUN_TIME = 10;

	// constants for timing of hopper movement during mining
	static constexpr int
		hopper_belt_mine_wait_time = 750,	// in milliseconds
		hopper_belt_mine_run_time = 100;	// in milliseconds

	static constexpr int
		TELEOP_LOW_SPEED_BUTTON_IDX = LogitechConstants::BUTTON_B,
		TELEOP_MEDIUM_SPEED_BUTTON_IDX = LogitechConstants::BUTTON_Y,
		TELEOP_HIGH_SPEED_BUTTON_IDX = LogitechConstants::BUTTON_X,
		
		TELEOP_DRIVE_X_AXIS_IDX = LogitechConstants::LEFT_JOY_X,
		TELEOP_DRIVE_Y_AXIS_IDX = LogitechConstants::LEFT_JOY_Y,

		TELEOP_TRENCHER_SPEED_AXIS_IDX = LogitechConstants::RIGHT_TRIGGER,
		TELEOP_TRENCHER_INVERT_IDX = LogitechConstants::RB,
		
		TELEOP_HOPPER_SPEED_AXIS_IDX = LogitechConstants::LEFT_TRIGGER,
		TELEOP_HOPPER_INVERT_IDX = LogitechConstants::LB,
		TELEOP_HOPPER_ACTUATE_AXIS_IDX = LogitechConstants::RIGHT_JOY_Y,

		TELEAUTO_MINING_INIT_POV = 0,
		TELEAUTO_MINING_STOP_POV = 90,
		TELEAUTO_OFFLOAD_INIT_POV = 180,
		TELEAUTO_OFFLOAD_STOP_POV = 270;

};
