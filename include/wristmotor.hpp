#ifndef WRISTMOTOR_H
#define WRISTMOTOR_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "motor.hpp"
#include <string>


class WristMotor : public Motor {

    private:
    public:

        WristMotor(uint8_t id) : Motor() {};
        ~WristMotor() = default;

        // WristMotor(const std::string& serial_port, int baudrate, float protocol_version, 
        //   dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
        //   uint8_t id);

        // WristMotor(const std::string& serial_port, int baudrate, float protocol_version,
        //   uint8_t id);

        // WristMotor();

};

#endif