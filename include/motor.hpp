#ifndef MOTOR_H
#define MOTOR_H

#include "dynamixel_sdk/dynamixel_sdk.h"

class Motor {
private:
    uint8_t id_;
    std::string serial_port_;
    int baudrate_;
    float protocol_version_;
    dynamixel::PortHandler *portHandler_; 
    dynamixel::PacketHandler *packetHandler_;

public:
    Motor(const std::string& serial_port, int baudrate, float protocol_version, 
          dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
          uint8_t id);
    
    Motor(const std::string& serial_port, int baudrate, float protocol_version,
          uint8_t id);

    Motor();



    // DISTRUTTORE?

    void setId(uint8_t id);
    int getId();

    void setPacketHandler(dynamixel::PacketHandler *packetHandler);
    void setPortHandler(dynamixel::PortHandler *portHandler);

    dynamixel::PortHandler* getPortHandler() const;
    dynamixel::PacketHandler* getPacketHandler() const;


    void write1OnAddress( uint16_t address, uint8_t data);
    void write2OnAddress(uint16_t address, uint16_t data);
    uint8_t read1FromAddress(uint16_t address);
    uint16_t read2FromAddress(uint16_t address);



    // setTargerPosition, getTargetPosition?, getPresentPosition, setSpeed, getSpeed .

};

#endif
