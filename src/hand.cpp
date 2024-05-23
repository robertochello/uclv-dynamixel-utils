#include "dynamixel_sdk/dynamixel_sdk.h"
#include <memory>
#include "my_library/hand.hpp"

/**
 * @brief Constructs a Hand object with the specified parameters.
 * 
 * @param serial_port The serial port to be used for communication.
 * @param baudrate The baudrate for serial communication.
 * @param protocol_version The protocol version to be used.
 * @param portHandler Pointer to the port handler.
 * @param packetHandler Pointer to the packet handler.
 */
Hand::Hand(const std::string& serial_port, int baudrate, float protocol_version, 
           dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler)
    : serial_port_(serial_port),
      baudrate_(baudrate),
      protocol_version_(protocol_version),
      portHandler_(portHandler),
      packetHandler_(packetHandler) {}

/**
 * @brief Constructs a Hand object with the specified parameters and initializes the port and packet handlers.
 * 
 * @param serial_port The serial port to be used for communication.
 * @param baudrate The baudrate for serial communication.
 * @param protocol_version The protocol version to be used.
 */
Hand::Hand(const std::string& serial_port, int baudrate, float protocol_version)
    : serial_port_(serial_port),
      baudrate_(baudrate),
      protocol_version_(protocol_version) {
    portHandler_ = dynamixel::PortHandler::getPortHandler(serial_port.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
}

/**
 * @brief Default constructor for Hand.
 */
Hand::Hand()
    : serial_port_(""),
      baudrate_(0),
      protocol_version_(0),
      portHandler_(nullptr),
      packetHandler_(nullptr) {}

/**
 * @brief Destructor for Hand. Cleans up the port and packet handlers.
 */
Hand::~Hand() {
    delete portHandler_;
    delete packetHandler_;
}

/**
 * @brief Initializes the serial port and sets the baudrate.
 */
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

/**
 * @brief Sets the serial port.
 * 
 * @param serial_port The serial port to be used for communication.
 */
void Hand::setSerialPort(const std::string& serial_port) {
    serial_port_ = serial_port;
}

/**
 * @brief Sets the baudrate.
 * 
 * @param baudrate The baudrate for serial communication.
 */
void Hand::setBaudrate(int baudrate) {
    baudrate_ = baudrate;
}

/**
 * @brief Sets the protocol version.
 * 
 * @param protocol_version The protocol version to be used.
 */
void Hand::setProtocolVersion(float protocol_version) {
    protocol_version_ = protocol_version;
}

/**
 * @brief Gets the serial port.
 * 
 * @return The serial port used for communication.
 */
std::string Hand::getSerialPort() {
    return serial_port_;
}

/**
 * @brief Gets the baudrate.
 * 
 * @return The baudrate used for serial communication.
 */
int Hand::getBaudrate() {
    return baudrate_;
}

/**
 * @brief Gets the protocol version.
 * 
 * @return The protocol version used.
 */
float Hand::getProtocolVersion() {
    return protocol_version_;
}

/**
 * @brief Sets the port handler.
 * 
 * @param portHandler Pointer to the port handler.
 */
void Hand::setPortHandler(dynamixel::PortHandler *portHandler) {
    portHandler_ = portHandler;
}

/**
 * @brief Sets the packet handler.
 * 
 * @param packetHandler Pointer to the packet handler.
 */
void Hand::setPacketHandler(dynamixel::PacketHandler *packetHandler) {
    packetHandler_ = packetHandler;
}

/**
 * @brief Sets the serial port to low latency mode.
 * 
 * @param serial_port The serial port to be used.
 */
void Hand::setSerialPortLowLatency(const std::string& serial_port) {
    std::cout << "Setting low latency for " << WARN_COLOR << serial_port << CRESET << std::endl;
    std::string command = "setserial " + serial_port + " low_latency";
    int result = system(command.c_str());
    std::cout << "Setting low latency for " << WARN_COLOR << serial_port << CRESET 
              << " result: " << WARN_COLOR << result << CRESET << std::endl;
}

/**
 * @brief Converts radians to degrees.
 * 
 * @param radians The angle in radians.
 * @return The angle in degrees.
 */
float Hand::rad_to_degrees(float radians) {
    return radians * (180.0 / M_PI);
}

/**
 * @brief Converts degrees to motor position.
 * 
 * @param degrees The angle in degrees.
 * @return The motor position.
 */
uint16_t Hand::degrees_to_position(float degrees) {
    const double conversion_factor = 0.088;
    return degrees / conversion_factor;
}

/**
 * @brief Converts degrees to radians.
 * 
 * @param degrees The angle in degrees.
 * @return The angle in radians.
 */
float Hand::degrees_to_rad(float degrees) {
    return degrees * (M_PI / 180.0);
}

/**
 * @brief Converts motor position to degrees.
 * 
 * @param position The motor position.
 * @return The angle in degrees.
 */
float Hand::position_to_degrees(uint16_t position) {
    const double conversion_factor = 0.088;
    return position * conversion_factor;
}

/**
 * @brief Gets the list of finger motors.
 * 
 * @return A vector of shared pointers to the finger motors.
 */
std::vector<std::shared_ptr<FingerMotor>> Hand::getFingerMotors() {
    return fingerMotors_;
}

/**
 * @brief Gets the list of wrist motors.
 * 
 * @return A vector of shared pointers to the wrist motors.
 */
std::vector<std::shared_ptr<WristMotor>> Hand::getWristMotors() {
    return wristMotors_;
}

/**
 * @brief Creates a finger motor with the specified ID.
 * 
 * @param id The ID of the finger motor.
 * @return A shared pointer to the created FingerMotor.
 */
std::shared_ptr<FingerMotor> Hand::createFingerMotor(uint8_t id) {
    return std::make_shared<FingerMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id);
}

/**
 * @brief Creates a wrist motor with the specified ID.
 * 
 * @param id The ID of the wrist motor.
 * @return A shared pointer to the created WristMotor.
 */
std::shared_ptr<WristMotor> Hand::createWristMotor(uint8_t id) {
    return std::make_shared<WristMotor>(serial_port_, baudrate_, protocol_version_, portHandler_, packetHandler_, id);
}

/**
 * @brief Adds a finger motor to the list if it does not already exist.
 * 
 * This function checks if a FingerMotor with the specified ID already exists in the list. 
 * If it does not, a new FingerMotor is created and added to the list.
 * 
 * @param id The ID of the FingerMotor to be added.
 * @return const std::vector<std::shared_ptr<FingerMotor>>& A reference to the list of FingerMotors.
 */
const std::vector<std::shared_ptr<FingerMotor>>& Hand::addFingerMotor(uint8_t id) {
    // Iterate through the existing finger motors to check if the motor with the specified ID already exists
    for (const auto& motor : fingerMotors_) {
        if (motor->getId() == id) {
            // If a motor with the specified ID is found, print a message and return the current list
            std::cout << "FingerMotor with ID " << id << " already exists in the list." << std::endl;
            return fingerMotors_;
        }
    }
    
    // If no motor with the specified ID is found, create a new FingerMotor and add it to the list
    fingerMotors_.push_back(createFingerMotor(id));
    return fingerMotors_;
}

/**
 * @brief Adds a wrist motor to the list if it does not already exist.
 * 
 * This function checks if a WristMotor with the specified ID already exists in the list. 
 * If it does not, a new WristMotor is created and added to the list.
 * 
 * @param id The ID of the WristMotor to be added.
 * @return const std::vector<std::shared_ptr<WristMotor>>& A reference to the list of WristMotors.
 */
const std::vector<std::shared_ptr<WristMotor>>& Hand::addWristMotor(uint8_t id) {
    // Iterate through the existing finger motors to check if the motor with the specified ID already exists
    for (const auto& motor : wristMotors_) {
        if (motor->getId() == id) {
            // If a motor with the specified ID is found, print a message and return the current list
            std::cout << "WristMotor with ID " << id << " already exists in the list." << std::endl;
            return



/**
 * @brief Adds a finger motor to the list if it does not already exist.
 * 
 * This function checks if a FingerMotor with the specified ID already exists in the list. 
 * If it does not, a new FingerMotor is created and added to the list.
 * 
 * @param id The ID of the FingerMotor to be added.
 * @return const std::vector<std::shared_ptr<FingerMotor>>& A reference to the list of FingerMotors.
 */
const std::vector<std::shared_ptr<FingerMotor>>& Hand::addFingerMotor(uint8_t id) {
    // Iterate through the existing finger motors to check if the motor with the specified ID already exists
    for (const auto& motor : fingerMotors_) {
        if (motor->getId() == id) {
            // If a motor with the specified ID is found, print a message and return the current list
            std::cout << "FingerMotor with ID " << id << " already exists in the list." << std::endl;
            return fingerMotors_;
        }
    }
    
    // If no motor with the specified ID is found, create a new FingerMotor and add it to the list
    fingerMotors_.push_back(createFingerMotor(id));
    return fingerMotors_;
}



/**
 * @brief Adds a wrist motor to the list if it does not already exist.
 * 
 * This function checks if a WristMotor with the specified ID already exists in the list. 
 * If it does not, a new WristMotor is created and added to the list.
 * 
 * @param id The ID of the WristMotor to be added.
 * @return const std::vector<std::shared_ptr<WristMotor>>& A reference to the list of WristMotors.
 */
const std::vector<std::shared_ptr<WristMotor>>& Hand::addWristMotor(uint8_t id) {
    // Iterate through the existing finger motors to check if the motor with the specified ID already exists
    for (const auto& motor : WristMotors_) {
        if (motor->getId() == id) {
            // If a motor with the specified ID is found, print a message and return the current list
            std::cout << "WristMotor with ID " << id << " already exists in the list." << std::endl;
            return wristMotors_;
        }
    }
    
    // If no motor with the specified ID is found, create a new FingerMotor and add it to the list
    wristMotors_.push_back(createWristMotor(id));
    return wristMotors_;
}






/**
 * @brief Prints the IDs of all finger motors.
 * 
 * This function prints the index and ID of each finger motor in the list. If no finger motors are found, it prints a message indicating this.
 */
void Hand::printFingerMotors() const {
    // Check if the list of finger motors is empty
    if (fingerMotors_.empty()) {
        std::cout << "No Finger Motors found." << std::endl;
        return;
    } else {
        std::cout << "Finger Motors:" << std::endl;
        // Iterate over the list of finger motors and print their index and ID
        for (size_t i = 0; i < fingerMotors_.size(); i++) {
            std::cout << "Index: " << i << " - ID: " << fingerMotors_[i]->getId() << std::endl;
        }
    }
}



/**
 * @brief Prints the IDs of all wrist motors.
 * 
 * This function prints the index and ID of each wrist motor in the list. If no wrist motors are found, it prints a message indicating this.
 */
void Hand::printWristMotors() const {
    // Check if the list of wrist motors is empty
    if (wristMotors_.empty()) {
        std::cout << "No Wrist Motors found." << std::endl;
        return;
    } else {
        std::cout << "Wrist Motors:" << std::endl;
        // Iterate over the list of wrist motors and print their index and ID
        for (size_t i = 0; i < wristMotors_.size(); i++) {
            std::cout << "Index: " << i << " - ID: " << wristMotors_[i]->getId() << std::endl;
        }
    }
}



/**
 * @brief Removes a finger motor from the list by its ID.
 * 
 * This function searches for a finger motor with the specified ID and removes it from the list of finger motors if found.
 * 
 * @param id The ID of the finger motor to remove.
 */
void Hand::removeFingerMotor(uint8_t id) {
    // Iterate over the list of finger motors
    for (auto i = fingerMotors_.begin(); i < fingerMotors_.end(); i++)  {
        // Check if the current motor's ID matches the specified ID
        if ((*i)->getId() == id) {
            // Erase the motor from the list and adjust the iterator
            i = fingerMotors_.erase(i);
            --i;  // Adjust iterator to prevent skipping elements
            std::cout << "ID: " << id << " removed." << std::endl;
        }
    }
}



/**
 * @brief Removes a wrist motor from the list by its ID.
 * 
 * This function searches for a wrist motor with the specified ID and removes it from the list of wrist motors if found.
 * 
 * @param id The ID of the wrist motor to remove.
 */
void Hand::removeWristMotor(uint8_t id) {
    // Iterate over the list of wrist motors
    for (auto i = wristMotors_.begin(); i < wristMotors_.end(); i++)  {
        // Check if the current motor's ID matches the specified ID
        if ((*i)->getId() == id) {
            // Erase the motor from the list and adjust the iterator
            i = wristMotors_.erase(i);
            --i;  // Adjust iterator to prevent skipping elements
            std::cout << "ID: " << id << " removed." << std::endl;
        }
    }
}

















/**
 * @brief Moves a finger motor to a specified position.
 * 
 * This function sets the target position for a FingerMotor identified by its ID. The position value 
 * should be within the range of 0 to 4095.
 * 
 * @param id The ID of the FingerMotor to be moved.
 * @param position The target position for the FingerMotor. It should be a value between 0 and 4095.
 */
void Hand::moveFingerMotor(const uint8_t& id, const float& position) {
    // Get the ID of the motor from the list of finger motors
    uint8_t id_motor = fingerMotors_[id]->getId();
    
    // Set the target position for the specified motor
    fingerMotors_[id]->setTargetPosition(id_motor, position);
}

/**
 * @brief Moves a wrist motor to a specified position.
 * 
 * This function sets the target position for a WristMotor identified by its ID. The position value 
 * should be within the range of 0 to 4095.
 * 
 * @param id The ID of the WristMotor to be moved.
 * @param position The target position for the WristMotor. It should be a value between 0 and 4095.
 */
void Hand::moveWristMotor(const uint8_t& id, const float& position) {
    // Get the ID of the motor from the list of finger motors
    uint8_t id_motor = wristMotors_Motors_[id]->getId();
    
    // Set the target position for the specified motor
    wristMotors_[id]->setTargetPosition(id_motor, position);
}









/**
 * @brief Reads the current position of a specified finger motor.
 * 
 * This function retrieves the current position of a FingerMotor identified by its ID.
 * 
 * @param id The ID of the FingerMotor whose position is to be read.
 * @return The current position of the FingerMotor. The position is a value between 0 and 4095.
 */
float Hand::readFingerPositionMotor(const uint8_t& id) {
    // Get the ID of the motor from the list of finger motors
    uint8_t id_motor = fingerMotors_[id]->getId();
    
    // Read the current position of the specified motor
    uint16_t position = fingerMotors_[id]->readPresentPosition(id_motor);
    
    // Return the current position
    return position;
}


/**
 * @brief Reads the current position of a specified wrist motor.
 * 
 * This function retrieves the current position of a WristMotor identified by its ID.
 * 
 * @param id The ID of the WristMotor whose position is to be read.
 * @return The current position of the WristMotor. The position is a value between 0 and 4095.
 */
float Hand::readWristPositionMotor(const uint8_t& id) {
    // Get the ID of the motor from the list of finger motors
    uint8_t id_motor = wristMotors_[id]->getId();
    
    // Read the current position of the specified motor
    uint16_t position = wristMotors_[id]->readPresentPosition(id_motor);
    
    // Return the current position
    return position;
}








/**
 * @brief Moves multiple motors to specified target positions using bulk write.
 * 
 * This function moves a set of motors (both finger and wrist motors) to specified positions in a single bulk write operation.
 * 
 * @param ids A vector of motor IDs to be moved.
 * @param positions A vector of target positions corresponding to each motor ID. Values should be in the range 0 to 4095.
 */
void Hand::moveMotors(const std::vector<uint16_t>& ids, const std::vector<int>& positions) {
    // Check if groupBulkWrite_ is initialized, if not, initialize it
    if (!groupBulkWrite_) {
        groupBulkWrite_ = std::make_unique<dynamixel::GroupBulkWrite>(portHandler_, packetHandler_);
    }

    // Check if the sizes of ids and positions vectors match
    if (ids.size() != positions.size()) {
        std::cerr << "Error: Mismatched IDs and positions sizes." << std::endl;
        return;
    }

    // Prepare parameters for bulk write
    uint8_t param_target_position[2];
    for (size_t i = 0; i < ids.size(); i++) {
        // Convert the position to low and high bytes
        param_target_position[0] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
        param_target_position[1] = DXL_HIBYTE(DXL_LOWORD(positions[i]));
        uint16_t motor_id = ids[i];

        bool motor_found = false;
        // Search for the motor in fingerMotors_
        for (const auto& motor : fingerMotors_) {
            if (motor->getId() == motor_id) {
                groupBulkWrite_->addParam(motor_id, 30, 2, param_target_position);
                motor_found = true;
                break;
            }
        }

        // If not found in fingerMotors_, search in wristMotors_
        if (!motor_found) {
            for (const auto& motor : wristMotors_) {
                if (motor->getId() == motor_id) {
                    groupBulkWrite_->addParam(motor_id, 30, 2, param_target_position);
                    motor_found = true;
                    break;
                }
            }
        }

        // If motor ID is not found in both lists, print an error and return
        if (!motor_found) {
            std::cerr << "Error: Motor ID " << motor_id << " not found." << std::endl;
            return;
        }
    }

    // Transmit the bulk write packet
    if (groupBulkWrite_->txPacket() != COMM_SUCCESS) {
        std::cerr << "Error: Failed to transmit bulk write packet." << std::endl;
    }
}








/**
 * @brief Reads the positions of multiple motors using bulk read.
 * 
 * This function reads the current positions of a set of motors (both finger and wrist motors) in a single bulk read operation.
 * 
 * @param ids A vector of motor IDs whose positions are to be read.
 * @return A vector of positions corresponding to each motor ID. The positions are in the range 0 to 4095.
 */
std::vector<float> Hand::readMotorsPositions(const std::vector<uint16_t>& ids) {
    std::vector<float> positions;

    // Check if groupBulkRead_ is initialized, if not, initialize it
    if (!groupBulkRead_) {
        groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);
    }
    groupBulkRead_->clearParam();

    // Add motors to bulk read
    for (size_t i = 0; i < ids.size(); i++) {
        uint16_t motor_id = ids[i];
        bool motor_found = false;

        // Search for the motor in fingerMotors_
        for (const auto& motor : fingerMotors_) {
            if (motor->getId() == motor_id) {
                if (!groupBulkRead_->addParam(motor_id, 30, 2)) {
                    std::cerr << "Failed to add FingerMotor ID " << motor_id << " to bulk read." << std::endl;
                }
                motor_found = true;
                break;
            }
        }

        // If not found in fingerMotors_, search in wristMotors_
        if (!motor_found) {
            for (const auto& motor : wristMotors_) {
                if (motor->getId() == motor_id) {
                    if (!groupBulkRead_->addParam(motor_id, 30, 2)) {
                        std::cerr << "Failed to add WristMotor ID " << motor_id << " to bulk read." << std::endl;
                    }
                    motor_found = true;
                    break;
                }
            }
        }

        // If motor ID is not found in both lists, print an error and return
        if (!motor_found) {
            std::cerr << "Error: Motor ID " << motor_id << " not found." << std::endl;
            return positions;
        }
    }

    // Transmit the bulk read packet
    if (groupBulkRead_->txRxPacket() != COMM_SUCCESS) {
        std::cerr << "Failed to execute bulk read." << std::endl;
        return positions;
    }

    // Retrieve the positions from the bulk read packet
    for (size_t i = 0; i < ids.size(); i++) {
        uint16_t motor_id = ids[i];
        bool motor_found = false;

        // Search for the motor in fingerMotors_
        for (const auto& motor : fingerMotors_) {
            if (motor->getId() == motor_id) {
                if (groupBulkRead_->isAvailable(motor_id, 30, 2)) {
                    int32_t position_data = groupBulkRead_->getData(motor_id, 30, 2);
                    positions.push_back(static_cast<float>(position_data));
                } else {
                    std::cerr << "Data not available for FingerMotor ID " << motor_id << "." << std::endl;
                }
                motor_found = true;
                break;
            }
        }

        // If not found in fingerMotors_, search in wristMotors_
        if (!motor_found) {
            for (const auto& motor : wristMotors_) {
                if (motor->getId() == motor_id) {
                    if (groupBulkRead_->isAvailable(motor_id, 30, 2)) {
                        int32_t position_data = groupBulkRead_->getData(motor_id, 30, 2);
                        positions.push_back(static_cast<float>(position_data));
                    } else {
                        std::cerr << "Data not available for WristMotor ID " << motor_id << "." << std::endl;
                    }
                    motor_found = true;
                    break;
                }
            }
        }

        // If motor ID is not found during data retrieval, print an error
        if (!motor_found) {
            std::cerr << "Error: Motor ID " << motor_id << " not found during data retrieval." << std::endl;
        }
    }

    return positions;
}
