#pragma once

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "colors.hpp"
#include <string>

namespace uclv::dynamixel_utils
{

    /**
     * @brief The Motor class represents a single motor connected to the system.
     */
    class Motor
    {
    private:
        uint8_t id_;              ///< Identifier of the motor.
        std::string serial_port_; ///< Serial port to which the motor is connected.
        int baudrate_;            ///< Baudrate for the serial communication.
        float protocol_version_;  ///< Protocol version used for communication.

        uint16_t addrTargetPosition_ = 30;  ///< Address for Target Position in RH8D's control table.
        uint16_t addrPresentPosition_ = 36; ///< Address for Present Position in RH8D's control table.

        uint16_t addrTorqueEnable_ = 24; ///< Address for Torque Enable in RH8D's control table.

        uint16_t lenAddrTargetPosition_ = 2;  ///< Size of address for Target Position in RH8D's control table.
        uint16_t lenAddrPresentPosition_ = 2; ///< Size of Address for Present Position in RH8D's control table.

        dynamixel::PortHandler *portHandler_;     ///< Pointer to the port handler.
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
        Motor(const std::string &serial_port, int baudrate, float protocol_version,
              dynamixel::PortHandler *const &portHandler, dynamixel::PacketHandler *const &packetHandler,
              uint8_t id);

        /**
         * @brief Constructs a Motor object with specified parameters.
         *
         * @param serial_port The serial port to which the motor is connected.
         * @param baudrate The baudrate for the serial communication.
         * @param protocol_version The protocol version used for communication.
         * @param id Identifier of the motor.
         */
        Motor(const std::string &serial_port, int baudrate, float protocol_version,
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
        uint8_t getId();

        /**
         * @brief Sets the address for the target position.
         *
         * @param addrTargetPosition The address to be set for the target position.
         */
        void setAddrTargetPosition(uint16_t addrTargetPosition);

        /**
         * @brief Gets the address for the target position.
         *
         * @return The address for the target position.
         */
        uint16_t getAddrTargetPosition() const;

        /**
         * @brief Sets the address for the present position.
         *
         * @param addrPresentPosition The address to be set for the present position.
         */
        void setAddrPresentPosition(uint16_t addrPresentPosition);

        /**
         * @brief Gets the address for the present position.
         *
         * @return The address for the present position.
         */
        uint16_t getAddrPresentPosition() const;

        /**
         * @brief Sets the size (nr of bytes to write) for address of the target position.
         *
         * @param lenAddrTargetPosition The size to be set for length of address for target position.
         */
        void setLenAddrTargetPosition(uint16_t lenAddrTargetPosition);

        /**
         * @brief Gets the size (nr of bytes to write) for address of the target position.
         *
         * @return  The size to be set for length of address for target position.
         */
        uint16_t getLenAddrTargetPosition() const;

        /**
         * @brief Sets the size (nr of bytes to write) for address of the present position.
         *
         * @param lenAddrPresentPosition The size to be set for length of address for present position.
         */
        void setLenAddrPresentPosition(uint16_t lenAddrPresentPosition);

        /**
         * @brief Gets the size (nr of bytes to write) for address of the present position.
         *
         * @return  The size to be set for length of address for present position.
         */
        uint16_t getLenAddrPresentPosition() const;

        void setAddrTorqueEnable(uint16_t addrTorqueEnable);

        uint16_t getAddrTorqueEnable() const;

        /**
         * @brief Sets the target position of the motor.
         *
         * @param position Target position.
         */
        void setTargetPosition(float position);

        /**
         * @brief Reads the present position of the motor.
         *
         * @return Present position of the motor.
         */
        uint16_t readPresentPosition();

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

        /**
         * @brief Enables torque for the motor.
         *
         * This method enables torque for the motor by writing a value of 1 to the torque enable address.
         *
         * @return True if torque is successfully enabled, false otherwise.
         */
        bool enableTorque();

        /**
         * @brief Disables torque for the motor.
         *
         * This method disables torque for the motor by writing a value of 0 to the torque enable address.
         *
         * @return True if torque is successfully disabled, false otherwise.
         */
        bool disableTorque();
    };
}