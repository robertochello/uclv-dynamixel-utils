#include "dynamixel_sdk/dynamixel_sdk.h"
#include "my_library/wristmotor.hpp"

/**
 * @brief Constructor of the WristMotor class.
 * 
 * This constructor initializes the WristMotor class with the provided parameters.
 * 
 * @param serial_port The serial port to which the motor is connected.
 * @param baudrate The baudrate of the serial communication.
 * @param protocol_version The version of the protocol used.
 * @param portHandler Pointer to the port handler.
 * @param packetHandler Pointer to the packet handler.
 * @param id The identifier of the motor.
 */
WristMotor::WristMotor(const std::string& serial_port, int baudrate, float protocol_version, 
                       dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
                       uint8_t id) 
    : Motor(serial_port, baudrate, protocol_version, portHandler, packetHandler, id) {}

/**
 * @brief Constructor of the WristMotor class.
 * 
 * This constructor initializes the WristMotor class with the provided parameters.
 * It creates the port and packet handlers based on the serial port and protocol version.
 * 
 * @param serial_port The serial port to which the motor is connected.
 * @param baudrate The baudrate of the serial communication.
 * @param protocol_version The version of the protocol used.
 * @param id The identifier of the motor.
 */
WristMotor::WristMotor(const std::string& serial_port, int baudrate, float protocol_version, uint8_t id)
    : Motor(serial_port, baudrate, protocol_version, id) {}

/**
 * @brief Default constructor of the WristMotor class.
 * 
 * This constructor initializes the WristMotor class with default values.
 */
WristMotor::WristMotor() : Motor() {}
