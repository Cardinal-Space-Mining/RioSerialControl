#include <list>
#include <time.h>

#include <frc/SerialPort.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>

#include "../../../PiSerialControl/include/SerialInterface.h"

using namespace std::chrono_literals;

/// @brief A state machine to run serial communications. We have to do it this way because too long of an opperation on the call back thread will crash the rio
class SerialMgr
{
private:
    enum class SerialStates
    {
        XON_WAIT,
        MSG_WAIT
    };

    // Member Variables
    SerialStates state;
    std::array<ctre::phoenix6::hardware::TalonFX,1> &motors;
    frc::SerialPort serial = frc::SerialPort(230400, frc::SerialPort::Port::kOnboard, 8, frc::SerialPort::Parity::kParity_None, frc::SerialPort::StopBits::kStopBits_One);
    bool enabled = false;
    time_t state_timestamp;

    static constexpr double timeout_s = 0.1;
    static constexpr const char XON = 0x11;
    static constexpr const char XOFF = 0x13;

    SerialResponse handle_motor_data_struct(const struct MotorDataStruct &mds);

    void xon_wait_state();
    void msg_wait_state();

    void change_state(SerialStates new_state);

public:
    SerialMgr(std::array<ctre::phoenix6::hardware::TalonFX,1> &mts) : motors(mts)
    {
        serial.SetTimeout(units::second_t(.1));
        serial.SetReadBufferSize(1); // Somewhat disceving name. Sets how many bytes must be collected before serial.read will return anything.
    };
    ~SerialMgr();

    void init();

    void serial_periodic();

    inline void enable()
    {
        enabled = true;
        state = SerialStates::XON_WAIT;
        serial.Reset();
    };

    inline void disable() { enabled = false; };
};
