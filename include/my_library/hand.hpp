#ifndef HAND_H
#define HAND_H

#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>

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


    // più shared pointer possono puntare allo stesso oggetto e l'oggetto verrà eliminato quando l'ultimo shared pointer esce dallo scope
    std::vector<std::shared_ptr<FingerMotor>> fingerMotors_;
    std::vector<std::shared_ptr<WristMotor>> wristMotors_;

    // può esserci solo uno unique_ptr che punta a quell'oggetto 
    std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_ = nullptr;
    std::unique_ptr<dynamixel::GroupSyncRead> groupSyncRead_ = nullptr;

    std::unique_ptr<dynamixel::GroupBulkRead> groupBulkRead_ = nullptr;
    std::unique_ptr<dynamixel::GroupBulkWrite> groupBulkWrite_ = nullptr;



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

    // distrutto portHandler e packetHandler
    ~Hand();



    void initialize();

    void setSerialPort(const std::string& serial_port);
    
    // NON DOVREBBE SERVIRE
    void setBaudrate(int baudrate);
    void setProtocolVersion(float protocol_version);
    std::string getSerialPort();
    int getBaudrate();
    float getProtocolVersion();

    void setPortHandler(dynamixel::PortHandler *portHandler);
    void setPacketHandler(dynamixel::PacketHandler *packetHandler);


    // ESISTONO GIA IN DYNAMIXEL
    // dynamixel::PortHandler* getPortHandler() const;
    // dynamixel::PacketHandler* getPacketHandler() const;


    // for stable communication with high Baudrate, USB latency values must be setted to the lower
    void setSerialPortLowLatency(const std::string& serial_port);


    float rad_to_degrees(float radians);
    uint16_t degrees_to_position(float degrees);

    float degrees_to_rad(float degrees);
    float position_to_degrees(uint16_t position);

    // shared pointer per la creazione del motore? forse no perc
    std::shared_ptr<FingerMotor> createFingerMotor(uint8_t id);
    std::shared_ptr<WristMotor> createWristMotor(uint8_t id);


    std::vector<std::shared_ptr<FingerMotor>> addFingerMotor(uint8_t id);    
    std::vector<std::shared_ptr<WristMotor>> addWristMotor(uint8_t id);

    void addMotor(std::shared_ptr<FingerMotor>& fingerMotor);
    void addMotor(std::shared_ptr<WristMotor>& wristMotor);
 

    std::vector<std::shared_ptr<FingerMotor>> getFingerMotors();
    std::vector<std::shared_ptr<WristMotor>> getWristMotors();


    void printFingerMotors() const;
    void printWristMotors() const;

    void printMotors() const;

    void removeFingerMotor(uint8_t id);
    void removeWristMotor(uint8_t id);






    // move
    void moveFingerMotor(const uint8_t& id, const float& position);
    void moveWristMotor(const uint8_t& id, const float& position);

    // read
    float readFingerPositionMotor(const uint8_t& id);
    float readWristPositionMotor(const uint8_t& id);





    // muove più motori con bulk
    void moveMotorsBulk(const std::vector<uint16_t>& ids, const std::vector<int>& positions);
    // leggi posizioni motori con bulk
    std::vector<float> readMotorsBulk(const std::vector<uint16_t>& ids);




};

#endif // HAND_HPP