#ifndef HAND_H
#define HAND_H

#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>

#include "dynamixel_sdk/dynamixel_sdk.h"

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

    std::vector<std::shared_ptr<FingerMotor>> fingerMotors_;
    std::vector<std::shared_ptr<WristMotor>> wristMotors_;


    std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_ = nullptr;
    std::unique_ptr<dynamixel::GroupSyncRead> groupSyncRead_ = nullptr;

    std::unique_ptr<dynamixel::GroupBulkRead> groupBulkRead_ = nullptr;
    std::unique_ptr<dynamixel::GroupBulkWrite> groupBulkWrite_ = nullptr;






public:



    enum CommMode{
        BulkWrite
    };

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

    // Destructor???? delete portHandler_ and packetHandler_ ???
    ~Hand() = default;



    void initialize();

    void setSerialPort(const std::string& serial_port);
    void setBaudrate(int baudrate);
    void setProtocolVersion(float protocol_version);
    std::string getSerialPort();
    int getBaudrate();
    float getProtocolVersion();

    void setPortHandler(dynamixel::PortHandler *portHandler);
    void setPacketHandler(dynamixel::PacketHandler *packetHandler);

    dynamixel::PortHandler* getPortHandler() const;
    dynamixel::PacketHandler* getPacketHandler() const;

    void setSerialPortLowLatency(const std::string& serial_port);

    std::unique_ptr<FingerMotor> createFingerMotor(uint8_t id);
    std::unique_ptr<WristMotor> createWristMotor(uint8_t id);


    void addFingerMotor(uint8_t id);    
    void addWristMotor(uint8_t id);

    void addMotor(std::shared_ptr<FingerMotor>& fingerMotor);
    void addMotor(std::shared_ptr<WristMotor>& wristMotor);

    void printFingerMotors() const;
    void printWristMotors() const;

    void printMotors() const;

    void removeFingerMotor(uint8_t id);
    void removeWristMotor(uint8_t id);









    // void setComMode( const  Hand::CommMode& mode )
    // {
    //     switch (mode)
    //     {
    //     case /* constant-expression */:
    //         /* code */
    //         break;
        
    //     default:
    //         break;
    //     }
    // }








    // void moveAsyncMotor(uint8_t id, double position)
    // {

    //     bulk.(....)
    // }

    // void asyncMove()
    // {
    //     bulk.write();
    // }







    // muove un motore
    void Hand::moveMotor(uint8_t id, float position);
    // muove più motori
    void Hand::moveMotors(const std::vector<uint16_t>& ids, const std::vector<double>& positions);
    // leggi posizione motore
    void Hand::readMotor(uint8_t id);
    // leggi posizione motori
    void Hand::readMotors(const std::vector<uint16_t>& ids);



    // muove un motore con bulk
    void Hand::moveMotorBulk(uint8_t id, float position);
    // muove più motori con bulk
    void Hand::moveMotorsBulk(const std::vector<uint16_t>& ids, const std::vector<double>& positions);
    // leggi posizione motori con bulk
    void Hand::readMotorBulk(uint8_t id);
    // check esistenza bulkRead e bulkWrite
    void Hand::checkBulk();  // NON CREDO SERVA




    // muove un motore con bulk usando addParamWrite (writeSingleBulk)
    void moveMotorBulk(uint8_t id, float position);
    // muove più motori con moveMotorBulk
    void moveMotorsBulk(const std::vector<uint16_t>& ids, const std::vector<double>& positions);

    void addParamWrite(uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t data);
    void writeSingleBulk(uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t data);




     //bisogna fare le varie read









































/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
    void addParamWrite(dynamixel::GroupBulkWrite *groupBulkWrite, uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t data);
    void addParamWrite(dynamixel::GroupSyncWrite *groupSyncWrite, uint8_t id, uint8_t data);
    void addParamRead(dynamixel::GroupBulkRead *groupBulkRead, uint8_t id, uint16_t start_address, uint16_t data_length);
    void addParamRead(dynamixel::GroupSyncRead *groupSyncRead, uint8_t id);

    void writeSingleSync(std::unique_ptr<dynamixel::GroupSyncWrite>& groupSyncWritePtr, Motor& motor, uint8_t data);
    void writeAllSync(std::unique_ptr<dynamixel::GroupSyncWrite>& groupSyncWritePtr, std::vector<Motor>& motors, uint8_t data);

    void readSingleSync(std::unique_ptr<dynamixel::GroupSyncRead>& groupSyncReadPtr, Motor& motor);
    void readAllSync(std::unique_ptr<dynamixel::GroupSyncRead>& groupSyncReadPtr, std::vector<Motor>& motors);

    void writeSingleBulk(std::unique_ptr<dynamixel::GroupBulkWrite>& groupBulkWritePtr, Motor& motor, uint16_t start_address, uint16_t data_length, uint8_t data);
    void readSingleBulk(std::unique_ptr<dynamixel::GroupBulkRead>& groupBulkReadPtr, Motor& motor, uint16_t start_address, uint16_t data_length);
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
};

#endif // HAND_HPP