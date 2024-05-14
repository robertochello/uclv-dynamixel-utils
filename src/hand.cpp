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


Hand::~Hand() 
    {
        delete portHandler_;
        delete packetHandler_;
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


void Hand::setSerialPortLowLatency(const std::string& serial_port) {
    std::cout << "Setting low latency for " << WARN_COLOR << serial_port << CRESET << std::endl;
    std::string command = "setserial " + serial_port + " low_latency";
    int result = system(command.c_str());
    std::cout << "Setting low latency for " << WARN_COLOR << serial_port << CRESET 
              << " result: " << WARN_COLOR << result << CRESET << std::endl;
}

float rad_to_angle(float radians) {
    return radians*(180.0/M_PI);
}

float angle_to_position(float angle) {
    const double conversion_factor = 0.088;
    return angle/conversion_factor;
}
















std::shared_ptr<FingerMotor> Hand::createFingerMotor(uint8_t id) {
    addFingerMotor(id);
    return std::make_shared<FingerMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id);
}


std::shared_ptr<WristMotor> Hand::createWristMotor(uint8_t id) {
    return std::make_shared<WristMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id);
    addWristMotor(id);
}



void Hand::addFingerMotor(uint8_t id) {
    for (const auto& motor : fingerMotors_) {
        if (motor->getId() == id) {
            // std::cout << "FingerMotor with ID " << id << " already exists in the list." << std::endl;
            return;
        }
    }
    fingerMotors_.push_back(std::make_shared<FingerMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id));
}


void Hand::addWristMotor(uint8_t id) {
    for (const auto& motor : wristMotors_) {
        if (motor->getId() == id) {
            // std::cout << "WristMotor with ID " << id << " already exists in the list." << std::endl;
            return;
        }
    }
    wristMotors_.push_back(std::make_shared<WristMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id));
}



void Hand::addMotor(std::shared_ptr<WristMotor>& wristMotor) {
    uint8_t id = wristMotor->getId();
    addWristMotor(id);
}


void Hand::addMotor(std::shared_ptr<FingerMotor>& fingerMotor) {
    uint8_t id = fingerMotor->getId();
    addFingerMotor(id);
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
    for (auto i = fingerMotors_.begin(); i < fingerMotors_.end(); i++)  {
        if ((*i)->getId() == id) {
            i = fingerMotors_.erase(i);
            --i;
            std::cout << "ID: " << id << " removed." << std::endl;
        } 
    }
}

void Hand::removeWristMotor(uint8_t id) {
    for (auto i = wristMotors_.begin(); i < wristMotors_.end(); i++)  {
        if ((*i)->getId() == id) {
            i = wristMotors_.erase(i);
            --i;
            std::cout << "ID: " << id << " removed." << std::endl;
        } 
    }
}



// muove un motore
// id del motore
// posizione
// chiama setTargetPosition -> chiama write2OnAddress (dentro ha write2ByteTxRx con parametro id)
// anche se uso finger o wrist va bene perché sono figli di motor
void Hand::moveMotor(const std::shared_ptr<Motor>& motor, const uint8_t& id, const float& position) {
    motor->setTargetPosition(position);
}



// muove più motori
// vettore ids
// vettore posizioni
// check ids e positions devono avere la stessa dimensione
// per ogni elemento di ids e positions che si trovano nella stessa posizione, chiama moveMotor
// anche se uso finger o wrist va bene perché sono figli di motor
void Hand::moveMotors(const std::vector<std::shared_ptr<Motor>>& motors, const std::vector<uint16_t>& ids, const std::vector<double>& positions) {
    if (ids.size() != positions.size())
    {
       // ERRORE DEVONO AVERE LA STESSA DIMENSIONE
    } 
    for (size_t i = 0; i < ids.size(); i++)
    {
        for (size_t j = 0; j < positions.size(); j++)
        {
            if (i == j) {
                for (const auto& motor : motors) {
                    moveMotor(motor, ids[i], positions[j]);
                }   
            }
        }    
    }  
}

// legge posizione motore
// id
// readPresentPosition() -> read2FromAddress() (dentro ha read2ByteTxRx con parametro id)
void Hand::readPositionMotor(const std::shared_ptr<Motor>& motor, const uint8_t& id) {
    auto position = motor->readPresentPosition();
    std::cout << "ID: [" << id << "] - Present position: " << position << std::endl;
}

// legge posizioni motori
// vettore ids
// readMotor(id) per ogni elemento del vettore
void Hand::readPositionsMotors(const std::vector<std::shared_ptr<Motor>>& motors, const std::vector<uint16_t>& ids) {
    for (const auto& motor : motors) {
        for (size_t i = 0; i < ids.size(); i++)
        {
            if (motor->getId() == ids[i]) {
                readPositionMotor(motor, ids[i]);
            }
        }
    }
}




// muove un motore con bulk
// id
// posizioni
// check esistenza bulk, se non c'è lo istanzia
// addParam bulkRead ha bisogno dell'id, address, length
// costruzione parametro addParam bulkWrite (posizione)
// addParam bulkWrite ha bisogno dell'id, address, length, parametro(posizione)
// txPacket
// const id e const position perché non vengono modificati nel metodo
void Hand::moveMotorBulk(const uint8_t& id, const float& position) {
    uint8_t param_target_position[2];
    if (!groupBulkWrite_)
    {
        groupBulkWrite_ = std::make_unique<dynamixel::GroupBulkWrite>(portHandler_, packetHandler_);
    }
    param_target_position[0] = DXL_LOBYTE(DXL_LOWORD(position));
    param_target_position[1] = DXL_LOBYTE(DXL_LOWORD(position));
    groupBulkWrite_->addParam(id, 30, 2, param_target_position);
    groupBulkWrite_->txPacket();
}



// muove più motori con bulk
// vettore ids
// vettore posizioni
// check esistenza bulk
// addParam bulkRead ha bisogno dell'id, address, length
// costruzione parametro addParam bulkWrite (posizione)
// addParam bulkWrite ha bisogno dell'id, address, length, parametro(posizione)
// txPacket
// const ids e const positions perché non vengono modificati nel metodo
void Hand::moveMotorsBulk(const std::vector<uint16_t>& ids, const std::vector<double>& positions) {
    uint8_t param_target_position[2];
    if (!groupBulkWrite_)
    {
        groupBulkWrite_ = std::make_unique<dynamixel::GroupBulkWrite>(portHandler_, packetHandler_);
    }
    for (size_t i = 0; i < positions.size(); i++)
    {
        param_target_position[0] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
        param_target_position[1] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
        groupBulkWrite_->addParam(ids[i], 30, 2, param_target_position);
    }
    groupBulkWrite_->txPacket();
}



// versione "normale"
// id
// check bulkRead
// addParam
// txRxPacket
// isAvailable
// getData
void Hand::readMotorBulk(const uint8_t& id) {
        if (!groupBulkRead_) {
            groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);
        }
        groupBulkRead_->addParam(id, 30, 2);
        groupBulkRead_->txRxPacket();
        groupBulkRead_->isAvailable(id, 30, 2);
        auto position = groupBulkRead_->getData(id, 30, 2);
        std::cout << "ID: [" << id << "] - Present position: " << position << std::endl;
}


void Hand::readMotorsBulk(const std::vector<uint16_t>& ids) {
    for (size_t i = 0; i < ids.size(); i++)
    {
        readMotorBulk(ids[i]);
    }
}









void Hand::moveMotorBulk2(const uint8_t& id, const float& position) {
    if (!groupBulkWrite_) {
        groupBulkWrite_ = std::make_unique<dynamixel::GroupBulkWrite>(portHandler_, packetHandler_);
    }
    addParamWrite(id, 30, 2, position);
    groupBulkWrite_->txPacket();
}

void Hand::moveMotorsBulk2(const std::vector<uint16_t>& ids, const std::vector<double>& positions) {
    for (size_t i = 0; i < ids.size(); i++)
        {
            for (size_t j = 0; j < positions.size(); j++)
            {
                if (i == j) {
                    moveMotorBulk2(ids[i], positions[j]); //in questo modo fa txPacket per ogni motore
                }
            }
        }
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












// position in radianti
void Hand::moveFinger(uint8_t id, float rad) {

    
}





