#ifndef FINGERMOTOR_H
#define FINGERMOTOR_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "motor.hpp"
#include <string>


class FingerMotor : public Motor {

    private:
    public:

        FingerMotor(uint8_t id) : Motor() {};
        ~FingerMotor() = default;
        
        // FingerMotor(const std::string& serial_port, int baudrate, float protocol_version, 
        //   dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
        //   uint8_t id);

        // FingerMotor(const std::string& serial_port, int baudrate, float protocol_version,
        //   uint8_t id);

        // FingerMotor();

};

#endif