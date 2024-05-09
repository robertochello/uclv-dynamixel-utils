#ifndef DEFINES_H
#define DEFINES_H


#define DEVICENAME                "/dev/ttyUSB0"     // Nome del dispositivo 
#define BAUDRATE                  1000000            // Baudrate
#define PROTOCOL_VERSION          2.0                // Protocol version

#define MAIN_BOARD                30                 // Main board
#define WIRST_ROTATION            31                 // Wirst rotation
#define WIRST_ADDUCTION           32                 // Wirst adduction
#define WIRST_FLEXION             33                 // Wirst flexion
#define THUMB_ADDUCTION           34                 // Thumb adduction
#define THUMB_FLEXION             35                 // Thumb flexion
#define INDEX_FLEXION             36                 // Index flexion
#define MIDDLE_FLEXION            37                 // Middle flexion
#define LAST_FLEXION              38                 // 4th and 5th flexion

#define ADDR_TORQUE_ENABLE        24                 // Torque enable
#define ADDR_TARGET_POSITION      30                 // Target position
#define ADDR_TARGET_SPEED         32                 // Target speed
#define ADDR_PRESENT_POSITION     36                 // Present position
#define ADDR_PRESENT_SPEED        38                 // Present speed
#define ADDR_PRESENT_TEMPERATURE  43                 // Present temperature

#define TORQUE_ENABLE             1                  // Torque enable value
#define TORQUE_DISABLE            0                  // Torque disable value

#define LEN_PRESENT_POSITION      2                  // data length for ADDR_PRESENT_POSITION
#define LEN_TARGET_POSITION       2                  // data length for ADDR_TARGET_POSITION
#define LEN_PRESENT_SPEED         2                  // data length for ADDR_PRESENT_SPEED
#define LEN_TARGET_SPEED          2                  // data length for ADDR_TARGET_SPEED
#define LEN_PRESENT_TEMPERATURE   1                  // data length for ADDR_PRESENT_TEMPERATURE


#endif