#ifndef MOTOR_H
#define MOTOR_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <string>

/**
 * @brief The Motor class represents a single motor connected to the system.
 */
class Motor {
private:
    uint8_t id_; ///< Identifier of the motor.
    std::string serial_port_; ///< Serial port to which the motor is connected.
    int baudrate_; ///< Baudrate for the serial communication.
    float protocol_version_; ///< Protocol version used for communication.
    
    dynamixel::PortHandler *portHandler_; ///< Pointer to the port handler.
    dynamixel::PacketHandler *packetHandler_; ///< Pointer to the packet handler.

public:
    /**
     * @brief Constructs a Motor object with specified parameters.
     * 
     * @param serial_port The serial port to which the motor is connected.
     * @param baudrate The baudrate for the serial communication.
     * @param protocol_version The protocol version used for communication.
     * @param portHandler Pointer to the port handler.
     * @param packetHandler Pointer to the packet handler.
     * @param id Identifier of the motor.
     */
    Motor(const std::string& serial_port, int baudrate, float protocol_version, 
          dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler,
          uint8_t id);
    
    /**
     * @brief Constructs a Motor object with specified parameters.
     * 
     * @param serial_port The serial port to which the motor is connected.
     * @param baudrate The baudrate for the serial communication.
     * @param protocol_version The protocol version used for communication.
     * @param id Identifier of the motor.
     */
    Motor(const std::string& serial_port, int baudrate, float protocol_version,
          uint8_t id);

    /**
     * @brief Default constructor.
     */
    Motor();

    // DISTRUTTORE?

    /**
     * @brief Sets the identifier of the motor.
     * 
     * @param id Identifier of the motor.
     */
    void setId(uint8_t id);

    /**
     * @brief Gets the identifier of the motor.
     * 
     * @return Identifier of the motor.
     */
    int getId();

    /**
     * @brief Sets the packet handler.
     * 
     * @param packetHandler Pointer to the packet handler.
     */
    void setPacketHandler(dynamixel::PacketHandler *packetHandler);

    /**
     * @brief Sets the port handler.
     * 
     * @param portHandler Pointer to the port handler.
     */
    void setPortHandler(dynamixel::PortHandler *portHandler);

    /**
     * @brief Gets the port handler.
     * 
     * @return Pointer to the port handler.
     */
    dynamixel::PortHandler* getPortHandler() const;

    /**
     * @brief Gets the packet handler.
     * 
     * @return Pointer to the packet handler.
     */
    dynamixel::PacketHandler* getPacketHandler() const;

    /**
     * @brief Sets the target position of the motor.
     * 
     * @param id Identifier of the motor.
     * @param position Target position.
     */
    void setTargetPosition(uint8_t id, float position);

    /**
     * @brief Reads the present position of the motor.
     * 
     * @param id Identifier of the motor.
     * @return Present position of the motor.
     */
    uint16_t readPresentPosition(uint8_t id);

    /**
     * @brief Writes a byte to the specified address of the motor.
     * 
     * @param id Identifier of the motor.
     * @param address Address to write to.
     * @param data Data to write.
     */
    void write1OnAddress(uint8_t id, uint16_t address, uint8_t data);

    /**
     * @brief Writes two bytes to the specified address of the motor.
     * 
     * @param id Identifier of the motor.
     * @param address Address to write to.
     * @param data Data to write.
     */
    void write2OnAddress(uint8_t id, uint16_t address, uint16_t data);

    /**
     * @brief Reads a byte from the specified address of the motor.
     * 
     * @param id Identifier of the motor.
     * @param address Address to read from.
     * @return Read data.
     */
    uint8_t read1FromAddress(uint8_t id, uint16_t address);

    /**
     * @brief Reads two bytes from the specified address of the motor.
     * 
     * @param id Identifier of the motor.
     * @param address Address to read from.
     * @return Read data.
     */
    uint16_t read2FromAddress(uint8_t id, uint16_t address);
};

#endif // MOTOR_H
