#ifndef DEFINES_H
#define DEFINES_H

// Serial device name
#define DEVICENAME                "/dev/ttyUSB0"

// Serial communication baudrate
#define BAUDRATE                  1000000

// Protocol version
#define PROTOCOL_VERSION          2.0

// Identifiers of various motors
#define MAIN_BOARD                30
#define WIRST_ROTATION            31
#define WIRST_ADDUCTION           32
#define WIRST_FLEXION             33
#define THUMB_ADDUCTION           34
#define THUMB_FLEXION             35
#define INDEX_FLEXION             36
#define MIDDLE_FLEXION            37
#define LAST_FLEXION              38

// Motor register addresses
#define ADDR_TORQUE_ENABLE        24
#define ADDR_TARGET_POSITION      30
#define ADDR_TARGET_SPEED         32
#define ADDR_PRESENT_POSITION     36
#define ADDR_PRESENT_SPEED        38
#define ADDR_PRESENT_TEMPERATURE  43

// Values to enable/disable motor control
#define TORQUE_ENABLE             1
#define TORQUE_DISABLE            0

// Data lengths for motor registers
#define LEN_PRESENT_POSITION      2
#define LEN_TARGET_POSITION       2
#define LEN_PRESENT_SPEED         2
#define LEN_TARGET_SPEED          2
#define LEN_PRESENT_TEMPERATURE   1

#endif // DEFINES_H
