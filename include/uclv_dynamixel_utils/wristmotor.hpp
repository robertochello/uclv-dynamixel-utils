#pragma once

#include "motor.hpp"

using namespace uclv::dynamixel_utils;

/**
 * @class WristMotor
 * @brief The WristMotor class represents a wrist motor, which is a specific type of motor.
 */
class WristMotor : public Motor {
public:

    /**
     * @brief Constructs a WristMotor object with specified parameters.
     * 
     * @param serial_port The serial port to which the motor is connected.
     * @param baudrate The baudrate for the serial communication.
     * @param protocol_version The protocol version used for communication.
     * @param portHandler Pointer to the port handler.
     * @param packetHandler Pointer to the packet handler.
     * @param id Identifier of the motor.
     */
    WristMotor(const std::string& serial_port, int baudrate, float protocol_version, 
               dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
               uint8_t id);

    /**
     * @brief Constructs a WristMotor object with specified parameters.
     * 
     * @param serial_port The serial port to which the motor is connected.
     * @param baudrate The baudrate for the serial communication.
     * @param protocol_version The protocol version used for communication.
     * @param id Identifier of the motor.
     */
    WristMotor(const std::string& serial_port, int baudrate, float protocol_version,
               uint8_t id);

    /**
     * @brief Default constructor.
     */
    WristMotor();
};
