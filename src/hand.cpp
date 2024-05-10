#include "dynamixel_sdk/dynamixel_sdk.h"

#include <memory>

#include "my_library/hand.hpp"




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











std::unique_ptr<FingerMotor> Hand::createFingerMotor(uint8_t id) {
    return std::make_unique<FingerMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id);
    addFingerMotor(id);
}


std::unique_ptr<WristMotor> Hand::createWristMotor(uint8_t id) {
    return std::make_unique<WristMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id);
    addWristMotor(id);
}



void Hand::addFingerMotor(uint8_t id) {
    for (const auto& motor : fingerMotors_) {
        if (motor->getId() == id) {
            // std::cout << "FingerMotor with ID " << id << " already exists in the list." << std::endl;
            return;
        }
    }
    fingerMotors_.push_back(std::make_unique<FingerMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id));
}


void Hand::addWristMotor(uint8_t id) {
    for (const auto& motor : wristMotors_) {
        if (motor->getId() == id) {
            // std::cout << "WristMotor with ID " << id << " already exists in the list." << std::endl;
            return;
        }
    }
    wristMotors_.push_back(std::make_unique<WristMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id));
}




void Hand::addMotor(std::shared_ptr<WristMotor>& wristMotor) {
    // check se il motor con quell'id è già presente nella lista
    wristMotors_.push_back(wristMotor);

}


void Hand::addMotor(std::shared_ptr<FingerMotor>& fingerMotor) {
    // check se il motor con quell'id è già presente nella lista
    fingerMotors_.push_back(fingerMotor);

}

// print tutti i motori
// printFingerMotors
// printWristMotors
void Hand::printMotors() const {
    printFingerMotors();
    printWristMotors();
}

void Hand::printFingerMotors() const {
    if (fingerMotors_.empty()) {
        std::cout << "No Finger Motors found." << std::endl;
        return;
    }
    std::cout << "Finger Motors:" << std::endl;
    for (const auto& motor : fingerMotors_) {
        std::cout << "ID: " << motor->getId() << std::endl;
    }
}


void Hand::printWristMotors() const {
    if (wristMotors_.empty()) {
        std::cout << "No Wrist Motors found." << std::endl;
        return;
    }
    std::cout << "Wrist Motors:" << std::endl;
    for (const auto& motor : wristMotors_) {
        std::cout << "ID: " << motor->getId() << std::endl;
    }
}


void Hand::removeFingerMotor(uint8_t id) {
    // elimina dalla lista il motore
}

void Hand::removeWristMotor(uint8_t id) {
    // elimina dalla lista il motore
}




// muove un motore
// id del motore
// posizione
// chiama setTargetPosition -> chiama write2OnAddress (dentro ha write2ByteTxRx con parametro id)
void Hand::moveMotor(uint8_t id, float position) {
    motor.setTargetPosition(position);
}


// muove più motori
// vettore ids
// vettore posizioni
// check ids e positions devono avere la stessa dimensione
// per ogni elemento di ids e positions che si trovano nella stessa posizione, chiama moveMotor
void Hand::moveMotors(const std::vector<uint16_t>& ids, const std::vector<double>& positions) {
    if (ids.size() != positions.size())
    {
       // ERRORE DEVONO AVERE LA STESSA DIMENSIONE
    } 
    for (size_t i = 0; i < ids.size(); i++)
    {
        for (size_t j = 0; j < positions.size(); j++)
        {
            if (i == j) {
                moveMotor(ids[i], positions[j]);
            }
        }    
    }  
}

// muove più motori con bulk
// vettore ids
// vettore posizioni
// check esistenza bulk
// addParam bulkRead ha bisogno dell'id, address, length
// costruzione parametro addParam bulkWrite (posizione)
// addParam bulkWrite ha bisogno dell'id, address, length, parametro(posizione)
// txPacket
void Hand::moveMotorsBulk(const std::vector<uint16_t>& ids, const std::vector<double>& positions) {
    uint8_t param_target_position[2];
    if (!groupBulkRead_ && !groupBulkWrite_) // qui serve solo il check per la write
    {
        groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);
        groupBulkWrite_ = std::make_unique<dynamixel::GroupBulkWrite>(portHandler_, packetHandler_);
    }
    for (size_t i = 0; i < positions.size(); i++)
    {
        param_target_position[0] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
        param_target_position[1] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
        groupBulkWrite_->addParam(ids[i], 30, 2, param_target_position);
    }
    groupBulkWrite_->txPacket();
    /*
        for (size_t i = 0; i < ids.size(); i++)
        {
            for (size_t j = 0; j < positions.size(); j++)
            {
                if (i == j) {
                    bulk(ids[i], positions[j]); //in questo modo fa txPacket per ogni motore
                }
            }
        }
    */
}


// muove un motore con bulk
// id
// posizioni
// check esistenza bulk, se non c'è lo istanzia
// addParam bulkRead ha bisogno dell'id, address, length
// costruzione parametro addParam bulkWrite (posizione)
// addParam bulkWrite ha bisogno dell'id, address, length, parametro(posizione)
// txPacket
void Hand::moveMotorBulk(uint8_t id, float position) {
    uint8_t param_target_position[2];
    if (!groupBulkRead_ && !groupBulkWrite_) // qui serve solo il check per la write
    {
        groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);
        groupBulkWrite_ = std::make_unique<dynamixel::GroupBulkWrite>(portHandler_, packetHandler_);
    }
    param_target_position[0] = DXL_LOBYTE(DXL_LOWORD(position));
    param_target_position[1] = DXL_LOBYTE(DXL_LOWORD(position));
    groupBulkWrite_->addParam(id, 30, 2, param_target_position);
    groupBulkWrite_->txPacket();
}




void Hand::checkBulk() {
    if (!groupBulkRead_ && !groupBulkWrite_)
    {
        groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);
        groupBulkWrite_ = std::make_unique<dynamixel::GroupBulkWrite>(portHandler_, packetHandler_);
    }
}










// legge posizione motore
// id
// readPresentPosition() -> read2FromAddress() (dentro ha read2ByteTxRx con parametro id)
void Hand::readMotor(uint8_t id) {
    auto position = Motor::readPresentPosition();
    std::cout << "ID: [" << id << "] - Present position: " << position << std::endl;
}

// legge posizioni motori
// vettore ids
// readMotor(id) per ogni elemento del vettore
void Hand::readMotors(const std::vector<uint16_t>& ids) {
    for (size_t i = 0; i < ids.size(); i++)
    {
        readMotor(ids[i]);
    }
}


// versione "normale"
// id
// check bulkRead
// addParam
// txRxPacket
// isAvailable
// getData
void Hand::readMotorBulk(uint8_t id) {
        if (!groupBulkRead_) {
            groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);
        }
        groupBulkRead_->addParam(id, 30, 2);
        groupBulkRead_->txRxPacket();
        groupBulkRead_->isAvailable(id, 30, 2);
        auto position = groupBulkRead_->getData(id, 30, 2);
        std::cout << "ID: [" << id << "] - Present position: " << position << std::endl;
}
































// trasformazione dato da scrivere
// addParam su bulkWrite
void Hand::addParamWrite(uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t data) {
    bool dxl_addparam_result = false;
    uint8_t param_target_position[2];
    param_target_position[0] = DXL_LOBYTE(DXL_LOWORD(data));
    param_target_position[1] = DXL_HIBYTE(DXL_LOWORD(data));
    dxl_addparam_result = groupBulkWrite_->addParam(id, start_address, data_length, param_target_position);
    if (!dxl_addparam_result) {
        std::cerr << "Failed to add param write for address: " << start_address << " for [ID:" << id << "]" << std::endl;
    }    
}

// inutile
void Hand::writeSingleBulk(uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t data) {
    addParamWrite(id, start_address, data_length, data);
}


void Hand::moveMotorBulk(uint8_t id, float position) {
    if (!groupBulkWrite_) {
        groupBulkWrite_ = std::make_unique<dynamixel::GroupBulkWrite>(portHandler_, packetHandler_);
    }
    addParamWrite(id, 30, 2, position);
    //writeSingleBulk(id, 30, 2, position);
    groupBulkWrite_->txPacket();
}




void Hand::moveMotorsBulk(const std::vector<uint16_t>& ids, const std::vector<double>& positions) {
    for (size_t i = 0; i < ids.size(); i++)
        {
            for (size_t j = 0; j < positions.size(); j++)
            {
                if (i == j) {
                    moveMotor(ids[i], positions[j]); //in questo modo fa txPacket per ogni motore
                }
            }
        }
}

















































void Hand::writeSingleSync(std::unique_ptr<dynamixel::GroupSyncWrite>& groupSyncWritePtr, Motor& motor, uint8_t data) {
    if (!groupSyncWritePtr) {
        // groupSyncWritePtr has not been initialized yet
        return;
    }
    int id = motor.getId();
    addParamWrite(groupSyncWritePtr.get(), id, data);
    groupSyncWritePtr->txPacket();
}

void Hand::writeAllSync(std::unique_ptr<dynamixel::GroupSyncWrite>& groupSyncWritePtr, std::vector<Motor>& motors, uint8_t data) {
    if (!groupSyncWritePtr) {
        // groupSyncWritePtr has not been initialized yet
        return;
    }
    for (auto& motor : motors) {
        writeSingleSync(groupSyncWritePtr, motor, data);
    }
    groupSyncWritePtr->clearParam();
}

void Hand::readSingleSync(std::unique_ptr<dynamixel::GroupSyncRead>& groupSyncReadPtr, Motor& motor) {
    if (!groupSyncReadPtr) {
        // groupSyncReadPtr has not been initialized yet
        return;
    }
    int id = motor.getId();
    addParamRead(groupSyncReadPtr.get(), id);
    // After writing parameters to the motor, txRxPacket, isAvailable, and getData
}

void Hand::readAllSync(std::unique_ptr<dynamixel::GroupSyncRead>& groupSyncReadPtr, std::vector<Motor>& motors) {
    if (!groupSyncReadPtr) {
        // groupSyncReadPtr has not been initialized yet 
        return;
    }
    for (auto& motor : motors) {
        readSingleSync(groupSyncReadPtr, motor);
    }
}

void Hand::writeSingleBulk(std::unique_ptr<dynamixel::GroupBulkWrite>& groupBulkWritePtr, Motor& motor, uint16_t start_address, uint16_t data_length, uint8_t data) {
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

void Hand::readSingleBulk(std::unique_ptr<dynamixel::GroupBulkRead>& groupBulkReadPtr, Motor& motor, uint16_t start_address, uint16_t data_length) {
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







