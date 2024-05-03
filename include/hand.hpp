#ifndef HAND_H
#define HAND_H

#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>

#include <dynamixel_sdk/dynamixel_sdk.h>

#include "colors.hpp"

#include "motor.hpp"
#include "fingermotor.hpp"
#include "wristmotor.hpp"




class Hand {

private:
    std::string serial_port_;
    int baudrate_;
    float protocol_version_;

    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;



public:
    /*
        const std::string& serial_port - il costruttore può accedere al nome della porta seriale senza effettuare una copia dell'oggetto
        dynamixel::PortHandler *const& portHandler - puntatore costante all'oggetto dynamixel::PortHandler, in questo modo il costruttore può
                                                    accedere all'oggetto portHandler senza modificarlo
        dynamixel::PacketHandler *const& packetHandler - come sopra
    */
    Hand(const std::string& serial_port, int baudrate, float protocol_version, 
            dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler);

    Hand(const std::string& serial_port, int baudrate, float protocol_version);

    Hand();

    // Destructor????
    ~Hand() = default;



    void initialize();

    void setSerialPort(const std::string& serial_port);
    void setBaudrate(int baudrate);
    void setProtocolVersion(float protocol_version);
    std::string getSerialPort();
    int getBaudrate();
    float getProtocolVersion();

    // questi servono per il primo tipo di costruttore: creo il costruttore senza gli handler, creo gli handler, setto gli handler
    void setPortHandler(dynamixel::PortHandler *portHandler);
    void setPacketHandler(dynamixel::PacketHandler *packetHandler);

    // sono const perché i metodi non modificano niente  ???????
    dynamixel::PortHandler* getPortHandler() const;
    dynamixel::PacketHandler* getPacketHandler() const;

    void setSerialPortLowLatency(const std::string& serial_port);

    // std::shared_ptr<FingerMotor> createFingerMotor(uint8_t id);
    std::shared_ptr<FingerMotor> createFingerMotor(uint8_t id);
    std::shared_ptr<WristMotor> createWristMotor(uint8_t id);

    
    void addFingerMotor(std::vector<std::shared_ptr<FingerMotor>>& fingerMotors, std::shared_ptr<FingerMotor> fingerMotor);
    void addWristMotor(std::vector<std::shared_ptr<WristMotor>>& wristMotors, std::shared_ptr<WristMotor> wristMotor);

    void removeFingerMotor(std::vector<FingerMotor>& fingerMotors, uint8_t id);
    void removeWristMotor(std::vector<WristMotor>& wristMotors, uint8_t id);

    std::shared_ptr<dynamixel::GroupSyncWrite> createWrite(uint16_t start_address, uint16_t data_length);
    std::shared_ptr<dynamixel::GroupBulkWrite> createWrite();
    std::shared_ptr<dynamixel::GroupSyncRead> createRead(uint16_t start_address, uint16_t data_length);
    std::shared_ptr<dynamixel::GroupBulkRead> createRead();

    void addParamWrite(dynamixel::GroupBulkWrite *groupBulkWrite, uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t data);
    void addParamWrite(dynamixel::GroupSyncWrite *groupSyncWrite, uint8_t id, uint8_t data);
    void addParamRead(dynamixel::GroupBulkRead *groupBulkRead, uint8_t id, uint16_t start_address, uint16_t data_length);
    void addParamRead(dynamixel::GroupSyncRead *groupSyncRead, uint8_t id);

    void writeSingleSync(std::shared_ptr<dynamixel::GroupSyncWrite>& groupSyncWritePtr, Motor& motor, uint8_t data);
    void writeAllSync(std::shared_ptr<dynamixel::GroupSyncWrite>& groupSyncWritePtr, std::vector<Motor>& motors, uint8_t data);

    void readSingleSync(std::shared_ptr<dynamixel::GroupSyncRead>& groupSyncReadPtr, Motor& motor);
    void readAllSync(std::shared_ptr<dynamixel::GroupSyncRead>& groupSyncReadPtr, std::vector<Motor>& motors);

    void writeSingleBulk(std::shared_ptr<dynamixel::GroupBulkWrite>& groupBulkWritePtr, Motor& motor, uint16_t start_address, uint16_t data_length, uint8_t data);
    void readSingleBulk(std::shared_ptr<dynamixel::GroupBulkRead>& groupBulkReadPtr, Motor& motor, uint16_t start_address, uint16_t data_length);

};

#endif // HAND_HPP