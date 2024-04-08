#include "SerialMgr.h"

#include <iostream>
#include <stdexcept>

using std::cout;
using std::endl;

namespace
{
  void default_cfg(ctre::phoenix6::hardware::TalonFX &m_fx)
  {
    ctre::phoenix6::configs::TalonFXConfiguration configs{};

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

    m_fx.GetConfigurator().Apply(configs);
  }

  void serial_write_bytes(frc::SerialPort &serial, const char *buffer, size_t buff_size)
  {
    cout << "serial write" << endl;
    do
    {
      int num_bytes_written = serial.Write(buffer, buff_size);
      buffer = (char *)((size_t)buffer + (size_t)num_bytes_written);
      buff_size -= num_bytes_written;
    } while (buff_size != 0);
    cout << "end of write" << endl;
  }

  void serial_read_bytes(frc::SerialPort &serial, char *buffer, size_t buff_size)
  {
    cout << "serial read" << endl;
    do
    {
      int num_bytes_read = serial.Read(buffer, buff_size);
      buffer = (char *)((size_t)buffer + (size_t)num_bytes_read);
      buff_size -= num_bytes_read;
    } while (buff_size != 0);
    cout << "end serial read" << endl;
  }

} // namespace


SerialMgr::~SerialMgr()
{
}

void SerialMgr::init()
{
  cout << "serialmgr init" << endl;
  for (auto &it : motors)
  {
    default_cfg(it);
  }
  cout << "end init" << endl;
}

SerialResponse SerialMgr::handle_motor_data_struct(const struct MotorDataStruct &mds)
{
  switch (mds.call_mode)
  {
  case MotorCallMode::PERCENT: // Percent Output
  {
    motors[mds.motor_number].Set(mds.percent);
    break;
  }
  case MotorCallMode::VELOCITY: // Velocity Output. See https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/cpp/VelocityClosedLoop/src/main/cpp/Robot.cpp
  {
    ctre::phoenix6::controls::VelocityVoltage m_voltageVelocity{0_tps, 0_tr_per_s_sq, true, 0_V, 0, false};
    motors[mds.motor_number].SetControl(m_voltageVelocity.WithVelocity(static_cast<units::angular_velocity::turns_per_second_t>(mds.velocity_turns_per_second)));
    break;
  }
  case MotorCallMode::DISABLE: // Disable Motor Mode
  {
    motors[mds.motor_number].Disable();
    break;
  }
  case MotorCallMode::NEUTRAL_MODE: // Set Neutral Mode
  {
    switch (mds.neutral_mode)
    {
    case MotorNeutralMode::MOTOR_COAST:
      motors[mds.motor_number].SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
      break;

    case MotorNeutralMode::MOTOR_BREAK:
      motors[mds.motor_number].SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
      break;
    default:
      return SerialResponse::INVALID_ARGUMENT;
    }
  }
  break;
  default:
    return SerialResponse::INVALID_ARGUMENT;
  }
  return SerialResponse::SUCCESS;
}

/// @brief We pull as many bytes as we can from the stream. If we hit XON, we do some things and switch to MSG_WAIT
void SerialMgr::xon_wait_state()
{
  // Read one byte at a time until we hit XON or run out of data
  char start_byte = 0;
  do
  {
    int num_bytes_read = serial.Read(&start_byte, sizeof(char));
    if (num_bytes_read != sizeof(char))
    {
      return;
    }

  } while (start_byte != XON);

  cout << "we have recieved the xon wait" << endl;

  serial_write_bytes(serial, &XON, sizeof(XON)); // response to being on
  change_state(SerialMgr::SerialStates::MSG_WAIT);
}

void SerialMgr::change_state(SerialStates new_state)
{
  state = new_state;
  state_timestamp = time(nullptr);
}

/// @brief We pull exactly as many bytes as we need from the stream. Then do some processing and switch to XON_WAIT
void SerialMgr::msg_wait_state()
{
  if (difftime(state_timestamp, time(nullptr)) > SerialMgr::timeout_s)
  {
    change_state(SerialStates::XON_WAIT);
    return;
  }

  // Read in Serial Message
  struct SerialMsg msg = {};

  // If we don't have enough bytes, wait for the rest to come in
  if (serial.GetBytesReceived() < (int)sizeof(SerialMsg))
  {
    return;
  }

  serial_read_bytes(serial, (char *)&msg, sizeof(SerialMsg));

  // Handle Message
  SerialResponse resp;
  try
  {
    switch (msg.type)
    {
    case SerialMsgType::MotorMessage:
      resp = handle_motor_data_struct(msg.mds);
      break;

    default:
      resp = SerialResponse::INVALID_ARGUMENT;
    }
  }
  catch (const std::exception&)
  {
    resp = SerialResponse::GENERAL_FAILURE;
  }
  catch (...)
  {
    resp = SerialResponse::GENERAL_FAILURE;
  }
  serial_write_bytes(serial, (char *)&resp, sizeof(resp)); // Send error code
  serial_write_bytes(serial, &XOFF, sizeof(XOFF));         // xoff, done running opcodes/commands. if panda doesnt get xoff, try opcode again
}

void SerialMgr::serial_periodic()
{
  // Don't run if not enabled
  if (!enabled)
  {
    return;
  }

  switch (state)
  {
  case SerialMgr::SerialStates::XON_WAIT:
    cout << "XON_WAIT" << endl;
    xon_wait_state();
    break;

  case SerialMgr::SerialStates::MSG_WAIT:
    cout << "MSG_WAIT" << endl;
    msg_wait_state();
    break;

  default:
    break;
  }
}