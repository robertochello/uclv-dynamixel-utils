#ifndef FINGERMOTOR_H
#define FINGERMOTOR_H

#include "motor.hpp"

/**
 * @brief The FingerMotor class represents a finger motor, which is a specific type of motor.
 */
class FingerMotor : public Motor {
private:
public:

    /**
     * @brief Constructs a FingerMotor object with specified parameters.
     * 
     * @param serial_port The serial port to which the motor is connected.
     * @param baudrate The baudrate for the serial communication.
     * @param protocol_version The protocol version used for communication.
     * @param portHandler Pointer to the port handler.
     * @param packetHandler Pointer to the packet handler.
     * @param id Identifier of the motor.
     */
    FingerMotor(const std::string& serial_port, int baudrate, float protocol_version, 
          dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
          uint8_t id);

    /**
     * @brief Constructs a FingerMotor object with specified parameters.
     * 
     * @param serial_port The serial port to which the motor is connected.
     * @param baudrate The baudrate for the serial communication.
     * @param protocol_version The protocol version used for communication.
     * @param id Identifier of the motor.
     */
    FingerMotor(const std::string& serial_port, int baudrate, float protocol_version,
          uint8_t id);

    /**
     * @brief Default constructor.
     */
    FingerMotor();

};

#endif // FINGERMOTOR_H
