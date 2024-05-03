#ifndef FINGERMOTOR_H
#define FINGERMOTOR_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "motor.hpp"
#include <string>


class FingerMotor : public Motor {

    private:
        uint8_t id_;
    public:
        FingerMotor(uint8_t id);
        ~FingerMotor();
};

#endif