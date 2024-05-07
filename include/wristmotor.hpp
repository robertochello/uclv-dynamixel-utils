#ifndef WRISTMOTOR_H
#define WRISTMOTOR_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "motor.hpp"
#include <string>


class WristMotor : public Motor {

    private:
        uint8_t id_;
    public:
        WristMotor(uint8_t id) : Motor() {};
        ~WristMotor() = default;
};

#endif