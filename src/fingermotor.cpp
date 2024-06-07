#include "dynamixel_sdk/dynamixel_sdk.h"
#include "uclv_dynamixel_utils/fingermotor.hpp"

using namespace uclv::dynamixel_utils;

/**
 * @class FingerMotor
 * @brief A class representing a finger motor, inheriting from the Motor class.
 */

/**
 * @brief Constructor of the FingerMotor class.
 * 
 * This constructor initializes the FingerMotor class with the provided parameters.
 * 
 * @param serial_port The serial port to which the motor is connected.
 * @param baudrate The baudrate of the serial communication.
 * @param protocol_version The version of the protocol used.
 * @param portHandler Pointer to the port handler.
 * @param packetHandler Pointer to the packet handler.
 * @param id The identifier of the motor.
 */
FingerMotor::FingerMotor(const std::string& serial_port, int baudrate, float protocol_version, 
                         dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
                         uint8_t id) 
    : Motor(serial_port, baudrate, protocol_version, portHandler, packetHandler, id) {}

/**
 * @brief Constructor of the FingerMotor class.
 * 
 * This constructor initializes the FingerMotor class with the provided parameters.
 * It creates the port and packet handlers based on the serial port and protocol version.
 * 
 * @param serial_port The serial port to which the motor is connected.
 * @param baudrate The baudrate of the serial communication.
 * @param protocol_version The version of the protocol used.
 * @param id The identifier of the motor.
 */
FingerMotor::FingerMotor(const std::string& serial_port, int baudrate, float protocol_version, uint8_t id)
    : Motor(serial_port, baudrate, protocol_version, id) {}

/**
 * @brief Default constructor of the FingerMotor class.
 * 
 * This constructor initializes the FingerMotor class with default values.
 */
FingerMotor::FingerMotor() : Motor() {}
