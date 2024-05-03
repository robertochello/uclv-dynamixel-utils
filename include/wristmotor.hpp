#ifndef WRISTMOTOR_H
#define WRISTMOTOR_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "motor.hpp"
#include <string>


class WristMotor : public Motor {

    private:
    
    public:
        WristMotor(uint8_t id);
        ~WristMotor();
};

#endif