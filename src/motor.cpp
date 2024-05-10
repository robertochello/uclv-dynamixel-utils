#include <iostream>

#include "my_library/motor.hpp"







Motor::Motor(const std::string& serial_port, int baudrate, float protocol_version, 
            dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
            uint8_t id)
    : 
    serial_port_(serial_port),
    baudrate_(baudrate),
    protocol_version_(protocol_version),
    portHandler_(portHandler),
    packetHandler_(packetHandler),
    id_(id) {}



Motor::Motor(const std::string& serial_port, int baudrate, float protocol_version,
             uint8_t id)
    :
    serial_port_(serial_port),
    baudrate_(baudrate),
    protocol_version_(protocol_version),
    id_(id)
    {
        portHandler_ = dynamixel::PortHandler::getPortHandler(serial_port.c_str());
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
    }


Motor::Motor()
    :
    serial_port_(""),
    baudrate_(0),
    protocol_version_(0),
    id_(0),
    portHandler_(nullptr),
    packetHandler_(nullptr) {}





void Motor::setId(uint8_t id) {
    id_ = id;
}


int Motor::getId() {
    return id_;
}

void Motor::setPortHandler(dynamixel::PortHandler *portHandler) {
    portHandler_ = portHandler;
}

void Motor::setPacketHandler(dynamixel::PacketHandler *packetHandler) {
    packetHandler_ = packetHandler;
}



dynamixel::PortHandler* Motor::getPortHandler() const {
    return portHandler_;
}

dynamixel::PacketHandler* Motor::getPacketHandler() const {
    return packetHandler_;
}






// setto posizione del motore
// gli passo la posizione in rad
// conversione rad -> position
// uso write2OnAddress
void Motor::setTargetPosition(float rad) { // forse la conversione la deve fare nel metodo MUOVI DITO
    angle = rad * 180 / PI
    position = angle / 0.088
    write2OnAddress(30, position);
}

// leggo posizione
// uso read2FromAddress
// conversione position -> rad
uint8_t Motor::readPresentPosition() {
    uint16_t position = read2FromAddress(30);
    angle = position * 0.088
    rad = angle * PI / 180
    return rad;
}





void Motor::write1OnAddress(uint16_t address, uint8_t data) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id_, address, data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        std::cerr << "Failed to write input: " << data << " with address: " << address << " for [ID:" << id_ << "]" << std::endl;
    }
    else {
        std::cout << "Success to write input: " << data << " with address: " << address << " for [ID:" << id_ << "]" << std::endl;
    }
}

void Motor::write2OnAddress(uint16_t address, uint16_t data) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id_, address, data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        std::cerr << "Failed to write input: " << data << " with address: " << address << " for [ID:" << id_ << "]" << std::endl;
    }
    else {
        std::cout << "Success to write input: " << data << " with address: " << address << " for [ID:" << id_ << "]" << std::endl;
    }
}

uint8_t Motor::read1FromAddress(uint16_t address) {
    uint8_t data;
    uint8_t dxl_error;
    int dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id_, address, &data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Failed to read from address: " << address << " for [ID:" << id_ << "]" << std::endl;
    }
    else {
        std::cout << "Success to read from address: " << address << " for [ID:" << id_ << "]" << std::endl;
    }
    return data;
}

uint16_t Motor::read2FromAddress(uint16_t address) {
    uint16_t data;
    uint8_t dxl_error;
    int dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id_, address, &data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Failed to read from address: " << address << " for [ID:" << id_ << "]" << std::endl;
    }
    else {
        std::cout << "Success to read from address: " << address << " for [ID:" << id_ << "]" << std::endl;
    }
    return data;
}

