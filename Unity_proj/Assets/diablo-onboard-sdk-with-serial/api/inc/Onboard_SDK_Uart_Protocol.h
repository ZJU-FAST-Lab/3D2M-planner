/**
 * @file Onboard_SDK_Uart_Protocol.h
 * @brief UART communication protocol
 */
#ifndef _ONBOARD_SDK_UART_PROTOCOL_H_
#define _ONBOARD_SDK_UART_PROTOCOL_H_

//VERSION: 1.0
#define OSDK_HEADER                                      0xAA
#define OSDK_VERSION                                       1U

#define OSDK_MISC_SIZE                                      4 //8bit cmd size, 8bit cmd id, 16bit CRC
#define OSDK_DATA_POS_OFFSET (sizeof(OSDK_Uart_Header_t) + 2)

#define OSDK_INIT_SET                0x00
#define OSDK_CONTROL_SET             0x01    
#define OSDK_DATA_SET                0x02
#define OSDK_VIRTUAL_RC_SET          0x05

#define OSDK_ACTIVATION_ID           0x01
#define OSDK_SET_PUSH_DATA_FREQ_ID   0x10

#define OSDK_CTRL_AUTHORIZE_ID       0x00   //Obtain/Release Control Authorization
#define OSDK_MOVEMENT_CTRL_ID        0x03 
#define OSDK_MOVEMENT_CTRL_MODE_ID   0x04
#define OSDK_TRANSFORM_ID            0x05
#define OSDK_EMERGENCY_BRAKE_ID      0x10   
#define OSDK_OFFLINE_CALIBRATION_ID  0x1F
#define OSDK_CONNECTION_CHECK_ID     0x2F

#define OSDK_PUSH_DATA_ID            0x00  

#define OSDK_VIRTUAL_RC_AUTHORIZE_ID 0x00   
#define OSDK_VIRTUAL_RC_DATA_ID      0x01

#define LEFT_HIP_DIR                 -1
#define LEFT_KNEE_DIR                -1
#define LEFT_WHEEL_DIR               1
#define RIGHT_HIP_DIR                1
#define RIGHT_KNEE_DIR               1
#define RIGHT_WHEEL_DIR              -1
#define DDJ_M15_MODE_MASK            0X07

#define CAN_RPM_2_RADIAN_SEC_PSC     1.04719755e-1f  
#define CAN_M15_RPM_PSC              1.f/109.f    // 2*M_PI / 0x2000
#define CAN_M15_IQ_PSC               6.103515625e-4f   

#define CTRL_MODE_CALIBRATION_1      127
#define CTRL_MODE_CALIBRATION_2      128

#define SDK_CONNECTED          0xA5

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief send request to robot
 * @note OSDK_ENABLE: enable request \n 
 *       OSDK_DISABLE: disable request \n
 */
#define OSDK_ENABLE   0xAA
#define OSDK_DISABLE  0x55

/**
 * @brief Header of packets
 * @note SOF: starting byte, fixed to be 0xAA \n
 *       LEN: len of frame (10 bit) \n
 *       VERSION: version of the frame header, set to be 0 (6 bit) \n
 *       SESSION: session ID, 0: Sender doesn't need ACKs. 1:Sender needs ACKs but can be tolerated. 2:Sender needs ACKs (5 bit) \n
 *       ACK: frame type. 0: CDM, 1: ACK (1 bit) \n
 *       SEQ: frame sequence number (2 byte) \n
 *       CRC16 : CRC16 frame header checksum (2 byte) \n
 */
typedef struct
{         
    uint8_t   SOF;          //starting byte, fixed to be 0xAA
    uint16_t  LEN     : 10; //len of frame
    uint16_t  VERSION :  6; //version of the frame header, set to be 0
    uint8_t   SESSION :  5; //session ID, 0: Sender doesn't need ACKs. 1:Sender needs ACKs but can be tolerated. 2:Sender needs ACKs.*
    uint8_t   ACK     :  1; //frame type, 0: CMD, 1:ACK
    uint8_t   RES0    :  2;
    uint8_t   PADDING :  5; //len of padding data used by the Data encryption
    uint8_t   ENC     :  3; //encryption type, 0: no encryption, 1: AES encryption
    uint8_t   RES1[3];
    uint16_t  SEQ;          //frame sequence num
    uint16_t  CRC16;        //CRC16 frame header checksum
}__attribute__((packed)) OSDK_Uart_Header_t;

/**
 * @brief UART communication packet structure
 * @note header: OSDK_Uart_Header_t \n
 *       CMD_SET: command set, 0x00: Initialization, 0x01: Control, 0x02: Push Data \n
 *       CMD_ID: command ID \n
 *       DATA: Pointer to data \n
 *       CRC16: CRC16 whole frame checksum \n
 */
typedef struct
{         
    OSDK_Uart_Header_t header;   //CRC16 frame header checksum
    uint8_t   CMD_SET;           //0x00, Initialization, 0x01, Control, 0x02, Push Data
    uint8_t   CMD_ID;
    void*     DATA;              //Pointer to data
    uint16_t  CRC16;             //CRC32 whole frame checksum
}__attribute__((packed)) OSDK_Uart_Packet_t;

/**
 * @brief ACK packet data part
 * @note cmd__set: equal to cmd set value of message that triggers ack \n
 *       cmd_id: equal to cmd id value of message that triggers ack \n
 *       VAL: ACK value \n
 */
typedef struct{
    uint8_t  cmd_set; //equal to cmd set value of message that triggers ack
    uint8_t  cmd_id;  //equal to cmd id value of message that triggers ack
    uint16_t VAL;
}__attribute__((packed)) OSDK_ACK_t;


/**
 * @brief ACK packet structure
 * @note header: OSDK_Uart_Header_t \n
 *       ACK: OSDK_ACK_t \n
 *       CRC16: CRC16 whole frame checksum \n
 */
typedef struct
{         
    OSDK_Uart_Header_t header;   //CRC16 frame header checksum
    OSDK_ACK_t  ACK;              //ACK value
    uint16_t  CRC16;            //CRC32 whole frame checksum
}__attribute__((packed)) OSDK_Uart_ACK_Packet_t;

typedef uint16_t OSDK_Offline_Calibration_Cmd_t;

/**
 * @brief Movement control mode data
 * @note head_controller_mode (1 bit), 1:head stable, 0: head assist balance \n
 *       yaw_ctrl_mode (1 bit), 1:control angle    0: control angular velocity \n
 *       split_ctrl_mode (1 bit), 1:control angle    0: control angular velocity \n
 *       pitch_ctrl_mode (1 bit), 1:control angle    0: control angular velocity \n
 *       roll_ctrl_mode (1 bit), 1:control angle    0: control angular velocity \n
 *       height_ctrl_mode (1 bit), 1:control height   0: control vertical velocity \n
 */
typedef struct
{
    uint16_t          reserve_mode : 11; //reserved for further use
    uint8_t   head_controller_mode : 1; //1:head stable      0: head assist balance

    uint8_t          yaw_ctrl_mode : 1; //1:control angle    0: control angular velocity
    uint8_t         split_ctrl_mode : 1; //1:control angle    0: control angular velocity
    uint8_t        pitch_ctrl_mode : 1; //1:control angle    0: control angular velocity
    uint8_t         roll_ctrl_mode : 1; //1:control angle    0: control angular velocity
    uint8_t       height_ctrl_mode : 1; //1:control height   0: control vertical velocity
}__attribute__((packed)) OSDK_Movement_Ctrl_Mode_t;

/**
 * @brief Movement control data (send speed or angle directly)
 */
typedef struct
{   
    float        forward;
    float           left;
    float             up;
    float           roll;
    float          pitch;
    float      leg_split;
}__attribute__((packed)) OSDK_Movement_Ctrl_t;

typedef struct
{   
    uint8_t       transform_up : 1;
    uint8_t     transform_down : 1;
    uint8_t            reserve : 6;
}__attribute__((packed)) OSDK_Transform_Cmd_t;

/**
 * @brief Movement control request
 * @note request: OSDK_ENABLE or OSDK_DISABLE \n
 *       timeout_act: Action after drop connection (1 bit), 1:Automatically switches to real remote control, 0 No switches，execute disconnect program \n
 *       timeout_ms: automatic release control authority after timeout ms (15 bit) \n
 */
typedef struct
{
    uint8_t request     :   1; //0xAA: Request ENABLE, 0x55: Request DISABLE
    uint8_t timeout_act :   1; //Action after drop connection, 1:Automatically switches to real remote control
                               //                              0 No switches，execute disconnect program

    uint16_t timeout_ms :  14; //automatic release control authority after timeout ms
}__attribute__((packed)) OSDK_Movement_Ctrl_Request_t;

/**
 * @brief Virtual remote controller request
 * @note request: (1 bit) 1: Request open, 0: Request close
 *       timeout_act: Action after drop connection (1 bit), 1:Automatically switches to real remote control, 0 No switches，execute disconnect program \n
 *       timeout_ms: automatic release control authority after timeout ms (14 bit) \n
 */
typedef struct{
    uint8_t  request     :  1; //1: Request open, 0: Request close
    uint8_t  timeout_act :  1; //Action after drop connection, 1:Automatically switches to real remote control
                               //                              0 No switches，execute disconnect program
    uint16_t timeout_ms  : 14; //automatic release control authority after timeout ms  
}__attribute__((packed)) OSDK_Virtual_RC_Request_t;

typedef struct
{
    uint16_t ch1; //channal value 200-1800
    uint16_t ch2;
    uint16_t ch3;
    uint16_t ch4;
    uint16_t ch5;
    uint16_t ch6;
    uint16_t ch7;
    uint16_t ch8;
    uint16_t ch9;
    uint16_t ch10;
    uint16_t ch11;
    uint16_t ch12;
    uint8_t  ch13     : 1;
    uint8_t  ch14     : 1;
    uint8_t  ch15     : 1;
    uint8_t  ch16     : 1;
    uint8_t  ch17     : 1;
    uint8_t  ch18     : 1;
    uint8_t  ch19     : 1;
    uint8_t  failsafe : 1;
}__attribute__((packed)) OSDK_Virtual_RC_t;

typedef uint8_t OSDK_Brake_Cmd_t;

typedef enum{
    OSDK_PUSH_DATA_OFF        = 0,
    OSDK_PUSH_DATA_1Hz        = 1,
    OSDK_PUSH_DATA_10Hz       = 2,
    OSDK_PUSH_DATA_50Hz       = 3,
    OSDK_PUSH_DATA_100Hz      = 4,
    OSDK_PUSH_DATA_500Hz      = 5,
    OSDK_PUSH_DATA_1000Hz     = 6,
    OSDK_PUSH_DATA_NO_CHANGE  = 7,
}OSDK_Push_Data_Freq_Select_t;

typedef struct{
    uint8_t save_config; //set 1 to save configuration to flash
    uint8_t status     ; //default 100Hz，frequency cannot be set below 100Hz, 
    uint8_t quaternion ; //default 0Hz
    uint8_t accl       ; //default 0Hz
    uint8_t gyro       ; //default 0Hz
    uint8_t RC         ; //default 0Hz
    uint8_t power      ; //default 1Hz
    uint8_t motor      ; //default 0Hz
    uint8_t reserve[6];
}__attribute__((packed)) OSDK_Set_Push_Data_Freq_t;

typedef struct{
    uint8_t  status      : 1; 
    uint8_t  quaternion  : 1; 
    uint8_t  accl        : 1;
    uint8_t  gyro        : 1;
    uint8_t  RC          : 1;
    uint8_t  power       : 1;
    uint8_t  motor       : 1;
    uint16_t reserve      : 9;
}__attribute__((packed)) OSDK_Push_Data_Flag_t;

/**
 * @brief Robot controll mode
 * @note  OSDK_CTRL_MODE_RC: Control by remote controller \n
 *        OSDK_CTRL_MODE_MOTION: Control by motion command \n
 *        OSDK_CTRL_MODE_VIRTUAL_RC: Control by virtual remote controller \n
 */
typedef enum{
    OSDK_CTRL_MODE_RC          = 0,
    OSDK_CTRL_MODE_MOTION      = 1,
    OSDK_CTRL_MODE_VIRTUAL_RC  = 2
}OSDK_Ctrl_Mode_t;

typedef enum{
    OSDK_ROBOT_STATE_DISCONNECT      = 0,
    OSDK_ROBOT_STATE_INITIALIZE      = 1,
    OSDK_ROBOT_STATE_CAR             = 2,
    OSDK_ROBOT_STATE_STAND           = 3,
    OSDK_ROBOT_STATE_TRANSITION_DOWN = 4,
    OSDK_ROBOT_STATE_TRANSITION_UP   = 5
}OSDK_Robot_State_t;

typedef uint32_t OSDK_Push_Data_Timestamp_t;

typedef struct{
    uint8_t  ctrl_mode;
    uint8_t  robot_mode;
    uint32_t error;
    uint32_t warning;
}__attribute__((packed)) OSDK_Push_Data_Status_t;

typedef struct{
    float w;
    float x;
    float y;
    float z;
}__attribute__((packed)) OSDK_Push_Data_Quaternion_t;

typedef struct{
    float x;
    float y;
    float z;
}__attribute__((packed)) OSDK_Push_Data_XYZ_t;

typedef struct{
    uint32_t ch1  : 11;
    uint32_t ch2  : 11;
    uint64_t ch3  : 11;
    uint32_t ch4  : 11;
    uint32_t ch5  : 11;
    uint32_t ch6  : 11;
    uint32_t ch7  : 11;
    uint32_t ch8  : 11;
    uint32_t ch9  : 11;
    uint32_t ch10 : 11;
    uint32_t ch11 : 11;
    uint32_t ch12 : 11;
    uint32_t ch13 : 11;
    uint32_t ch14 : 11;
    uint32_t ch15 : 11;
    uint32_t ch16 : 11;
    uint8_t  ch17 : 1;
    uint8_t  ch18 : 1;
    uint8_t  frame_lost : 1;
    uint8_t  failsafe   : 1;
    uint8_t  reserve    : 4;
} __attribute__((packed)) OSDK_Push_Data_RC_t;

typedef struct{
    float   voltage;
    float   current;
    float   capacitor_energy;
    uint8_t power_percent;
}__attribute__((packed)) OSDK_Push_Data_Power_t;

typedef struct{
    int32_t enc_rev;
    int16_t round_pos;
    int16_t ang_vel;
    int16_t iq;
}__attribute__((packed)) OSDK_Push_Data_M15_t;

typedef struct{
    OSDK_Push_Data_M15_t left_hip;
    OSDK_Push_Data_M15_t left_knee;
    OSDK_Push_Data_M15_t left_wheel;
    OSDK_Push_Data_M15_t right_hip;
    OSDK_Push_Data_M15_t right_knee;
    OSDK_Push_Data_M15_t right_wheel;
}__attribute__((packed)) OSDK_Push_Data_Motor_t;

#ifdef __cplusplus
}
#endif

#endif
