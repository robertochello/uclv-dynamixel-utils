#ifndef FINGERMOTOR_H
#define FINGERMOTOR_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "motor.hpp"


class FingerMotor : public Motor {

    private:
    public:


        
        FingerMotor(const std::string& serial_port, int baudrate, float protocol_version, 
          dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
          uint8_t id);

        FingerMotor(const std::string& serial_port, int baudrate, float protocol_version,
          uint8_t id);

        FingerMotor();

};

#endif