#include "dynamixel_sdk/dynamixel_sdk.h"

#include <memory>

#include "fingermotor.hpp"


// FingerMotor::FingerMotor(const std::string& serial_port, int baudrate, float protocol_version, 
//             dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
//             uint8_t id) 
//             : Motor(serial_port, baudrate, protocol_version, portHandler, packetHandler, id) {}

// FingerMotor::FingerMotor(const std::string& serial_port, int baudrate, float protocol_version, uint8_t id)
//             : Motor(serial_port, baudrate, protocol_version, id) {}

FingerMotor::FingerMotor() : Motor() {};