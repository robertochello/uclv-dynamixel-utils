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




float Hand::rad_to_degrees(float radians) {
    return radians*(180.0/M_PI);
}
uint16_t Hand::degrees_to_position(float degrees) {
    const double conversion_factor = 0.088;
    return degrees/conversion_factor;
}

float Hand::degrees_to_rad(float degrees) {
    return degrees*(M_PI/180.0);
}
float Hand::position_to_degrees(uint16_t position) {
    const double conversion_factor = 0.088;
    return position*conversion_factor;
}






std::vector<std::shared_ptr<FingerMotor>> Hand::getFingerMotors() {
    return fingerMotors_;
}

std::vector<std::shared_ptr<WristMotor>> Hand::getWristMotors() {
    return wristMotors_;
}




std::shared_ptr<FingerMotor> Hand::createFingerMotor(uint8_t id) {
    addFingerMotor(id);
    return std::make_shared<FingerMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id);
}


std::shared_ptr<WristMotor> Hand::createWristMotor(uint8_t id) {
    addWristMotor(id);
    return std::make_shared<WristMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id);
}



std::vector<std::shared_ptr<FingerMotor>> Hand::addFingerMotor(uint8_t id) {
    for (const auto& motor : fingerMotors_) {
        if (motor->getId() == id) {
            std::cout << "FingerMotor with ID " << id << " already exists in the list." << std::endl;
            return fingerMotors_;
        }
    }
    fingerMotors_.push_back(std::make_shared<FingerMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id));
    return fingerMotors_;
}


std::vector<std::shared_ptr<WristMotor>> Hand::addWristMotor(uint8_t id) {
    for (const auto& motor : wristMotors_) {
        if (motor->getId() == id) {
            std::cout << "WristMotor with ID " << id << " already exists in the list." << std::endl;
            return wristMotors_;
        }
    }
    wristMotors_.push_back(std::make_shared<WristMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id));
    return wristMotors_;
}




void Hand::printMotors() const {
    printFingerMotors();
    printWristMotors();
}

void Hand::printFingerMotors() const {
    if (fingerMotors_.empty()) {
        std::cout << "No Finger Motors found." << std::endl;
        return;
    } else
    {
        std::cout << "Finger Motors:" << std::endl;
        for (size_t i = 0; i < fingerMotors_.size(); i++)
        {
            std::cout << "Index: " << i <<" - ID: " << fingerMotors_[i]->getId() << std::endl;
        }
    }
}


void Hand::printWristMotors() const {
    if (wristMotors_.empty()) {
        std::cout << "No Wrist Motors found." << std::endl;
        return;
    } else
    {
        std::cout << "Wrist Motors:" << std::endl;
        for (size_t i = 0; i < wristMotors_.size(); i++)
        {
            std::cout << "Index: " << i <<" - ID: " << wristMotors_[i]->getId() << std::endl;
        }
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
















void Hand::moveFingerMotor(const uint8_t& id, const float& position) {
    uint8_t id_motor = fingerMotors_[id]->getId(); 
    float degrees = rad_to_degrees(position);
    uint16_t x = degrees_to_position(degrees);
    fingerMotors_[id]->setTargetPosition(id_motor, x);
}
void Hand::moveWristMotor(const uint8_t& id, const float& position) {
    uint8_t id_motor = wristMotors_[id]->getId();
    wristMotors_[id]->setTargetPosition(id_motor, position);
}








float Hand::readFingerPositionMotor(const uint8_t& id) {
    uint8_t id_motor = fingerMotors_[id]->getId();
    uint16_t position = fingerMotors_[id]->readPresentPosition(id_motor);
    float degrees = position_to_degrees(position);
    float rad = degrees_to_rad(degrees);
    return rad;
}

float Hand::readWristPositionMotor(const uint8_t& id) {
    uint8_t id_motor = wristMotors_[id]->getId();
    uint16_t position = wristMotors_[id]->readPresentPosition(id_motor);
    float degrees = position_to_degrees(position);
    float rad = degrees_to_rad(degrees);
    return rad;
}







void Hand::moveMotorsBulk(const std::vector<uint16_t>& ids, const std::vector<int>& positions) {
    uint8_t param_target_position[2];
    uint8_t motor_id;
    if (!groupBulkWrite_)
    {   
        groupBulkWrite_ = std::make_unique<dynamixel::GroupBulkWrite>(portHandler_, packetHandler_);
    }
    if (ids.size() != positions.size()) {
        return;
    }
    else
    {   
        for (size_t i = 0; i < positions.size(); i++)
        {   
            motor_id = fingerMotors_[ids[i]]->getId();
            param_target_position[0] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
            param_target_position[1] = DXL_HIBYTE(DXL_LOWORD(positions[i]));
            if (motor_id > 33 && motor_id < 39) {
                groupBulkWrite_->addParam(fingerMotors_[ids[i]]->getId(), 30, 2, param_target_position);
            } else if (ids[i] > 30 && ids[i] < 34)
            {
                groupBulkWrite_->addParam(wristMotors_[ids[i]]->getId(), 30, 2, param_target_position);
            } else return;  //ERRORE
            
        }
        groupBulkWrite_->txPacket();
    }
}





// TEST
std::vector<float> Hand::readMotorsBulk(const std::vector<uint16_t>& ids) {
    std::vector<float> positions;
    uint8_t motor_id;
    if (!groupBulkRead_) {
        groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);
    }
    for (size_t i = 0; i < ids.size(); i++)
    {
        if (ids[i] > 33 && ids[i] < 39) {
                motor_id = fingerMotors_[ids[i]]->getId();
                groupBulkRead_->addParam(motor_id, 30, 2);
            } else if (ids[i] > 30 && ids[i] < 34)
            {
                motor_id = wristMotors_[ids[i]]->getId();
                groupBulkRead_->addParam(motor_id, 30, 2);
            } else exit;  //ERRORE
    }
    groupBulkRead_->txRxPacket();
    for (size_t i = 0; i < ids.size(); i++)
    {
        if (ids[i] > 33 && ids[i] < 39) {
                motor_id= fingerMotors_[ids[i]]->getId();
                groupBulkRead_->isAvailable(motor_id, 30, 2);
                float position = groupBulkRead_->getData(motor_id, 30, 2);
                positions.push_back(position);
            } else if (ids[i] > 30 && ids[i] < 34)
            {
                motor_id = wristMotors_[ids[i]]->getId();
                groupBulkRead_->isAvailable(motor_id, 30, 2);
                float position = groupBulkRead_->getData(motor_id, 30, 2);
                positions.push_back(position);
            } else exit;  //ERRORE
    }
    return positions;
}
