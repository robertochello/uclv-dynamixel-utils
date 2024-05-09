#include "dynamixel_sdk/dynamixel_sdk.h"

#include <memory>

#include "my_library/wristmotor.hpp"


WristMotor::WristMotor(const std::string& serial_port, int baudrate, float protocol_version, 
            dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
            uint8_t id) 
            : Motor(serial_port, baudrate, protocol_version, portHandler, packetHandler, id) {}

WristMotor::WristMotor(const std::string& serial_port, int baudrate, float protocol_version, uint8_t id)
            : Motor(serial_port, baudrate, protocol_version, id) {}

WristMotor::WristMotor() : Motor() {};