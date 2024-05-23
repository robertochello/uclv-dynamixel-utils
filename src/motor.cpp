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
    id_(id) {}

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
    id_(id)
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
    packetHandler_(nullptr) {}

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
 * @brief Sets the port handler for the motor.
 * 
 * @param portHandler Pointer to the new port handler.
 */
void Motor::setPortHandler(dynamixel::PortHandler *portHandler) {
    portHandler_ = portHandler;
}

/**
 * @brief Sets the packet handler for the motor.
 * 
 * @param packetHandler Pointer to the new packet handler.
 */
void Motor::setPacketHandler(dynamixel::PacketHandler *packetHandler) {
    packetHandler_ = packetHandler;
}

/**
 * @brief Gets the port handler of the motor.
 * 
 * @return Pointer to the port handler.
 */
dynamixel::PortHandler* Motor::getPortHandler() const {
    return portHandler_;
}

/**
 * @brief Gets the packet handler of the motor.
 * 
 * @return Pointer to the packet handler.
 */
dynamixel::PacketHandler* Motor::getPacketHandler() const {
    return packetHandler_;
}

/**
 * @brief Sets the target position for the motor.
 * 
 * @param id The identifier of the motor.
 * @param position The target position to set.
 */
void Motor::setTargetPosition(uint8_t id, float position) {
    write2OnAddress(id, 30, position);
}

/**
 * @brief Reads the present position of the motor.
 * 
 * @param id The identifier of the motor.
 * @return The present position of the motor.
 */
uint16_t Motor::readPresentPosition(uint8_t id) {
    return read2FromAddress(id, 30);
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
        std::cerr << "Failed to write input: " << data << " with address: " << address << " for [ID:" << id << "]" << std::endl;
    }
    else {
        std::cout << "Success to write input: " << data << " with address: " << address << " for [ID:" << id << "]" << std::endl;
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
        std::cerr << "Failed to write input: " << data << " with address: " << address << " for [ID:" << id << "]" << std::endl;
    }
    else {
        std::cout << "Success to write input: " << data << " with address: " << address << " for [ID:" << id << "]" << std::endl;
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
        std::cerr << "Failed to read from address: " << address << " for [ID:" << id << "]" << std::endl;
    }
    else {
        std::cout << "Success to read from address: " << address << " for [ID:" << id << "]" << std::endl;
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
        std::cerr << "Failed to read from address: " << address << " for [ID:" << id << "]" << std::endl;
    }
    else {
        std::cout << "Success to read from address: " << address << " for [ID:" << id << "]" << std::endl;
    }
    return data;
}
