#include "dynamixel_sdk/dynamixel_sdk.h"

#include <memory>

#include "hand.hpp"



/*==============================================================================
                                  Constructors
==============================================================================*/
Hand::Hand(const std::string& serial_port, int baudrate, float protocol_version, 
            dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler)
    : 
    serial_port_(serial_port),
    baudrate_(baudrate),
    protocol_version_(protocol_version),
    portHandler_(portHandler),
    packetHandler_(packetHandler) {}


Hand::Hand(const std::string& serial_port, int baudrate, float protocol_version)
    :
    serial_port_(serial_port),
    baudrate_(baudrate),
    protocol_version_(protocol_version) 
    {
        portHandler_ = dynamixel::PortHandler::getPortHandler(serial_port.c_str());
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
    }


Hand::Hand()
    :
    serial_port_(""),
    baudrate_(0),
    protocol_version_(0),
    portHandler_(nullptr),
    packetHandler_(nullptr) {}




/*==============================================================================
                                  Set - Get
==============================================================================*/

void Hand::setSerialPort(const std::string& serial_port) {
    serial_port_ = serial_port;
}

void Hand::setBaudrate(int baudrate) {
    baudrate_ = baudrate;
}

void Hand::setProtocolVersion(float protocol_version) {
    protocol_version_ = protocol_version;
}

std::string Hand::getSerialPort() {
    return serial_port_;
}

int Hand::getBaudrate() {
    return baudrate_;
}

float Hand::getProtocolVersion() {
    return protocol_version_;
}

void Hand::setPortHandler(dynamixel::PortHandler *portHandler) {
    portHandler_ = portHandler;
}

void Hand::setPacketHandler(dynamixel::PacketHandler *packetHandler) {
    packetHandler_ = packetHandler;
}

dynamixel::PortHandler* Hand::getPortHandler() const {
    return portHandler_;
}

dynamixel::PacketHandler* Hand::getPacketHandler() const {
    return packetHandler_;
}

void Hand::setSerialPortLowLatency(const std::string& serial_port) {
    std::cout << "Setting low latency for " << WARN_COLOR << serial_port << CRESET << std::endl;
    std::string command = "setserial " + serial_port + " low_latency";
    int result = system(command.c_str());
    std::cout << "Setting low latency for " << WARN_COLOR << serial_port << CRESET 
              << " result: " << WARN_COLOR << result << CRESET << std::endl;
}






void Hand::initialize() {
    if (portHandler_ == nullptr || packetHandler_ == nullptr) {
        std::cerr << "ERROR: portHandler or packetHandler not initialized properly." << std::endl;
        return;
    }
    if (!portHandler_->openPort()) {
        std::cerr << "ERROR: serial port not opened." << std::endl;
    }
    if (!portHandler_->setBaudRate(baudrate_)) {
        std::cerr << "ERROR: baudrate not set." << std::endl;
    }
    std::cout << "Connection initialized successfully." << std::endl;
}







std::shared_ptr<FingerMotor> Hand::createFingerMotor(uint8_t id) {
    if(!portHandler_)
    {
        throw std::runtime_error("aujdgasdgas");
    }
    std::shared_ptr<FingerMotor> newFingerMotor = std::make_shared<FingerMotor>(id);
    return newFingerMotor;
}

std::shared_ptr<WristMotor> Hand::createWristMotor(uint8_t id) {
    std::shared_ptr<WristMotor> newWristMotor = std::make_unique<WristMotor>(id);
    return newWristMotor;
}



void Hand::addFingerMotor(std::vector<std::shared_ptr<FingerMotor>>& fingerMotors, std::shared_ptr<FingerMotor> fingerMotor) {
    fingerMotors.push_back(fingerMotor);
}


void Hand::addWristMotor(std::vector<std::shared_ptr<WristMotor>>& wristMotors, std::shared_ptr<WristMotor> wristMotor) {
    wristMotors.push_back(std::move(wristMotor));
}



void Hand::removeFingerMotor(std::vector<FingerMotor>& fingerMotors, uint8_t id) {
    for (auto it = fingerMotors.begin(); it != fingerMotors.end(); ++it) {
        if (it->getId() == id) {
            fingerMotors.erase(it);
            std::cout << "FingerMotor [ID:" << id << "] removed successfully." << std::endl;
            return;
        }
    }
    std::cerr << "FingerMotor [ID:" << id << "] not found." << std::endl;
}

void Hand::removeWristMotor(std::vector<WristMotor>& wristMotors, uint8_t id) {
    for (auto it = wristMotors.begin(); it != wristMotors.end(); ++it) {
        if (it->getId() == id) {
            wristMotors.erase(it);
            std::cout << "WristMotor [ID:" << id << "] removed successfully." << std::endl;
            return;
        }
    }
    std::cerr << "WristMotor [ID:" << id << "] not found." << std::endl;
}






std::shared_ptr<dynamixel::GroupSyncWrite> Hand::createWrite(uint16_t start_address, uint16_t data_length) {
    return std::make_unique<dynamixel::GroupSyncWrite>(portHandler_, packetHandler_, start_address, data_length);
}


std::shared_ptr<dynamixel::GroupBulkWrite> Hand::createWrite() {
    return std::make_unique<dynamixel::GroupBulkWrite>(portHandler_, packetHandler_);
}


std::shared_ptr<dynamixel::GroupSyncRead> Hand::createRead(uint16_t start_address, uint16_t data_length) {
    return std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, start_address, data_length);
}


std::shared_ptr<dynamixel::GroupBulkRead> Hand::createRead() {
    return std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);
}






void Hand::writeSingleSync(std::shared_ptr<dynamixel::GroupSyncWrite>& groupSyncWritePtr, Motor& motor, uint8_t data) {
    if (!groupSyncWritePtr) {
        // groupSyncWritePtr has not been initialized yet
        return;
    }
    int id = motor.getId();
    addParamWrite(groupSyncWritePtr.get(), id, data);
    groupSyncWritePtr->txPacket();
}

void Hand::writeAllSync(std::shared_ptr<dynamixel::GroupSyncWrite>& groupSyncWritePtr, std::vector<Motor>& motors, uint8_t data) {
    if (!groupSyncWritePtr) {
        // groupSyncWritePtr has not been initialized yet
        return;
    }
    for (auto& motor : motors) {
        writeSingleSync(groupSyncWritePtr, motor, data);
    }
    groupSyncWritePtr->clearParam();
}

void Hand::readSingleSync(std::shared_ptr<dynamixel::GroupSyncRead>& groupSyncReadPtr, Motor& motor) {
    if (!groupSyncReadPtr) {
        // groupSyncReadPtr has not been initialized yet
        return;
    }
    int id = motor.getId();
    addParamRead(groupSyncReadPtr.get(), id);
    // After writing parameters to the motor, txRxPacket, isAvailable, and getData
}

void Hand::readAllSync(std::shared_ptr<dynamixel::GroupSyncRead>& groupSyncReadPtr, std::vector<Motor>& motors) {
    if (!groupSyncReadPtr) {
        // groupSyncReadPtr has not been initialized yet 
        return;
    }
    for (auto& motor : motors) {
        readSingleSync(groupSyncReadPtr, motor);
    }
}

void Hand::writeSingleBulk(std::shared_ptr<dynamixel::GroupBulkWrite>& groupBulkWritePtr, Motor& motor, uint16_t start_address, uint16_t data_length, uint8_t data) {
    if (!groupBulkWritePtr) {
        // groupBulkWritePtr has not been initialized yet
        return;
    }
    int id = motor.getId();
    addParamWrite(groupBulkWritePtr.get(), id, start_address, data_length, data);
    groupBulkWritePtr->txPacket();
    // Optionally clear the parameters in the GroupBulkWrite instance
    // groupBulkWritePtr->clearParam();
}

void Hand::readSingleBulk(std::shared_ptr<dynamixel::GroupBulkRead>& groupBulkReadPtr, Motor& motor, uint16_t start_address, uint16_t data_length) {
    if (!groupBulkReadPtr) {
        // groupBulkReadPtr has not been initialized yet
        return;
    }
    int id = motor.getId();
    addParamRead(groupBulkReadPtr.get(), id, start_address, data_length);
    // After adding parameters to the motor, you can perform read operations such as txRxPacket, isAvailable, and getData
}





void Hand::addParamRead(dynamixel::GroupBulkRead *groupBulkRead, uint8_t id, uint16_t start_address, uint16_t data_length) {
    bool dxl_addparam_result = false;
    dxl_addparam_result = groupBulkRead->addParam(id, start_address, data_length);
    if (!dxl_addparam_result) {
        std::cerr << "Failed to add param for address: " << start_address << " for [ID:" << id << "]" << std::endl;
    }
}

void Hand::addParamRead(dynamixel::GroupSyncRead *groupSyncRead, uint8_t id) {
    bool dxl_addparam_result = false;
    dxl_addparam_result = groupSyncRead->addParam(id);
    if (!dxl_addparam_result) {
        std::cerr << "Failed to add param for [ID:" << id << "]" << std::endl;
    }
}

void Hand::addParamWrite(dynamixel::GroupBulkWrite *groupBulkWrite, uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t data) {
    bool dxl_addparam_result = false;
    dxl_addparam_result = groupBulkWrite->addParam(id, start_address, data_length, &data);
    if (!dxl_addparam_result) {
        std::cerr << "Failed to add param for address: " << start_address << " for [ID:" << id << "]" << std::endl;
    }    
}

void Hand::addParamWrite(dynamixel::GroupSyncWrite *groupSyncWrite, uint8_t id, uint8_t data) {
    bool dxl_addparam_result = false;
    dxl_addparam_result = groupSyncWrite->addParam(id, &data);
    if (!dxl_addparam_result) {
        std::cerr << "Failed to add param for [ID:" << id << "]" << std::endl;
    }    
}
