#include <iostream>
#include "my_library/motor.hpp"

/**
 * @brief Constructor of the Motor class.
 * 
 * This constructor initializes the Motor class with the provided parameters.
 * 
 * @param serial_port The serial port to which the motor is connected.
 * @param baudrate The baudrate of the serial communication.
 * @param protocol_version The version of the protocol used.
 * @param portHandler Pointer to the port handler.
 * @param packetHandler Pointer to the packet handler.
 * @param id The identifier of the motor.
 */
Motor::Motor(const std::string& serial_port, int baudrate, float protocol_version, 
             dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
             uint8_t id)
    : 
    serial_port_(serial_port),
    baudrate_(baudrate),
    protocol_version_(protocol_version),
    portHandler_(portHandler),
    packetHandler_(packetHandler),
    id_(id),
    addrTargetPosition_(30),
    addrPresentPosition_(36),
    addrTorqueEnable_(24),
    lenAddrTargetPosition_(2),
    lenAddrPresentPosition_(2) {}

/**
 * @brief Constructor of the Motor class.
 * 
 * This constructor initializes the Motor class with the provided parameters.
 * It creates the port and packet handlers based on the serial port and protocol version.
 * 
 * @param serial_port The serial port to which the motor is connected.
 * @param baudrate The baudrate of the serial communication.
 * @param protocol_version The version of the protocol used.
 * @param id The identifier of the motor.
 */
Motor::Motor(const std::string& serial_port, int baudrate, float protocol_version,
             uint8_t id)
    :
    serial_port_(serial_port),
    baudrate_(baudrate),
    protocol_version_(protocol_version),
    id_(id),
    addrTargetPosition_(30),
    addrPresentPosition_(36),
    addrTorqueEnable_(24),
    lenAddrTargetPosition_(2),
    lenAddrPresentPosition_(2)
    {
        portHandler_ = dynamixel::PortHandler::getPortHandler(serial_port.c_str());
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
    }

/**
 * @brief Default constructor of the Motor class.
 * 
 * This constructor initializes the Motor class with default values.
 */
Motor::Motor()
    :
    serial_port_(""),
    baudrate_(0),
    protocol_version_(0),
    id_(0),
    portHandler_(nullptr),
    packetHandler_(nullptr),
    addrTargetPosition_(30),
    addrPresentPosition_(36),
    addrTorqueEnable_(24),
    lenAddrTargetPosition_(2),
    lenAddrPresentPosition_(2) {}

/**
 * @brief Sets the ID of the motor.
 * 
 * @param id The new identifier of the motor.
 */
void Motor::setId(uint8_t id) {
    id_ = id;
}

/**
 * @brief Gets the ID of the motor.
 * 
 * @return The identifier of the motor.
 */
int Motor::getId() {
    return id_;
}

/**
 * @brief Sets the address for the target position.
 * 
 * @param addrTargetPosition The address to be set for the target position.
 */
void Motor::setAddrTargetPosition(uint16_t addrTargetPosition) {
    addrTargetPosition_ = addrTargetPosition;
}

/**
 * @brief Gets the address for the target position.
 * 
 * @return The address for the target position.
 */
uint16_t Motor::getAddrTargetPosition() const {
    return addrTargetPosition_;
}

/**
 * @brief Sets the address for the present position.
 * 
 * @param addrPresentPosition The address to be set for the present position.
 */
void Motor::setAddrPresentPosition(uint16_t addrPresentPosition) {
    addrPresentPosition_ = addrPresentPosition;
}

/**
 * @brief Gets the address for the present position.
 * 
 * @return The address for the present position.
 */
uint16_t Motor::getAddrPresentPosition() const {
    return addrPresentPosition_;
}

/**
 * @brief Sets the size (nr of bytes to write) for address of the target position.
 * 
 * @param lenAddrTargetPosition The size to be set for length of address for target position.
 */
void Motor::setLenAddrTargetPosition(uint16_t lenAddrTargetPosition) {
    lenAddrTargetPosition_ = lenAddrTargetPosition;
}

/**
 * @brief Gets the size (nr of bytes to write) for address of the target position.
 * 
 * @return  The size to be set for length of address for target position.
 */
uint16_t Motor::getLenAddrTargetPosition() const {
    return lenAddrTargetPosition_;
}

/**
 * @brief Sets the size (nr of bytes to write) for address of the present position.
 * 
 * @param lenAddrPresentPosition The size to be set for length of address for present position.
 */
void Motor::setLenAddrPresentPosition(uint16_t lenAddrPresentPosition) {
    lenAddrPresentPosition_ = lenAddrPresentPosition;
}

/**
 * @brief Gets the size (nr of bytes to write) for address of the present position.
 * 
 * @return  The size to be set for length of address for present position.
 */
uint16_t Motor::getLenAddrPresentPosition() const {
    return lenAddrPresentPosition_;
}




void Motor::setAddrTorqueEnable(uint16_t addrTorqueEnable) {
    addrTorqueEnable_ = addrTorqueEnable;
}

uint16_t Motor::getAddrTorqueEnable() const {
    return addrTorqueEnable_;
}

/**
 * @brief Sets the target position for the motor.
 * 
 * @param id The identifier of the motor.
 * @param position The target position to set.
 */
void Motor::setTargetPosition(uint8_t id, float position) {
    write2OnAddress(id, getAddrTargetPosition(), position);
}

/**
 * @brief Reads the present position of the motor.
 * 
 * @param id The identifier of the motor.
 * @return The present position of the motor.
 */
uint16_t Motor::readPresentPosition(uint8_t id) {
    return read2FromAddress(id, getAddrPresentPosition());
}

/**
 * @brief Writes a 1-byte value to a specified address on the motor.
 * 
 * @param id The identifier of the motor.
 * @param address The address to write the data to.
 * @param data The data to write.
 */
void Motor::write1OnAddress(uint8_t id, uint16_t address, uint8_t data) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, address, data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        std::cerr << "Failed to write input: " << data << " with address: " << address << " for [ID:" << (unsigned int)id << "]" << std::endl;
    }
    else {
        std::cout << "Success to write input: " << data << " with address: " << address << " for [ID:" << (unsigned int)id << "]" << std::endl;
    }
}

/**
 * @brief Writes a 2-byte value to a specified address on the motor.
 * 
 * @param id The identifier of the motor.
 * @param address The address to write the data to.
 * @param data The data to write.
 */
void Motor::write2OnAddress(uint8_t id, uint16_t address, uint16_t data) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, address, data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        std::cerr << "Failed to write input: " << data << " with address: " << address << " for [ID:" << (unsigned int)id << "]" << std::endl;
    }
    else {
        std::cout << "Success to write input: " << data << " with address: " << address << " for [ID:" << (unsigned int)id << "]" << std::endl;
    }
}

/**
 * @brief Reads a 1-byte value from a specified address on the motor.
 * 
 * @param id The identifier of the motor.
 * @param address The address to read the data from.
 * @return The data read from the address.
 */
uint8_t Motor::read1FromAddress(uint8_t id, uint16_t address) {
    uint8_t data;
    uint8_t dxl_error;
    int dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, address, &data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Failed to read from address: " << address << " for [ID:" << (unsigned int)id << "]" << std::endl;
    }
    else {
        std::cout << "Success to read from address: " << address << " for [ID:" << (unsigned int)id << "]" << std::endl;
    }
    return data;
}

/**
 * @brief Reads a 2-byte value from a specified address on the motor.
 * 
 * @param id The identifier of the motor.
 * @param address The address to read the data from.
 * @return The data read from the address.
 */
uint16_t Motor::read2FromAddress(uint8_t id, uint16_t address) {
    uint16_t data;
    uint8_t dxl_error;
    int dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, address, &data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Failed to read from address: " << address << " for [ID:" << (unsigned int)id << "]" << std::endl;
    }
    else {
        std::cout << "Success to read from address: " << address << " for [ID:" << (unsigned int)id << "]" << std::endl;
    }
    return data;
}




bool Motor::enableTorque() {
  uint8_t dxl_error = 0;
  int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, getId(), getAddrTorqueEnable(), 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        std::cerr << "Failed to enable torque for motor ID: " << getId() << std::endl;
        return false;
    }
    else {
        std::cerr << "Torque enable for motor ID: " << getId() << std::endl;
    }
    return true;
}


bool Motor::disableTorque() {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, getId(), getAddrTorqueEnable(), 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        std::cerr << "Failed to disable torque for motor ID: " << getId() << std::endl;
        return false;
    }
    else {
        std::cerr << "Torque disabled for motor ID: " << getId() << std::endl;
    }
    return true;
}