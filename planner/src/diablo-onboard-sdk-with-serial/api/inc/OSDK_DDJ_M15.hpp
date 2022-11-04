#pragma once

#include <stdio.h>
#include <iostream>
#include "Onboard_SDK_Uart_Protocol.h"

typedef enum {
    U_LOW1             = 1, 
    U_LOW2		       = 2, 
    U_UP               = 3, 
    U_D_LOW            = 4, 
    I_UP               = 10, 
    I_STEP             = 11, 
    V_UP               = 20, 
    T_UP               = 30, 
    T_UP_2             = 31,
    T_UP_1             = 32, 
    S_ISHUNT           = 41, 
    S_ANGLE_SENSOR     = 42,
    S_ANGLE_Disturb    = 43, 
    S_TEMP_UP_LOW      = 45, 
    S_TEMP_FAIL        = 46,
    S_TEMP_SHORT_GND   = 47,
    C_CAN_FAIL         = 60, 
    C_EEPROM_FAIL      = 61, 
    R_EEPROM_FAIL      = 70,
    M_PHASE_LOSS       = 80, 
    M_PHASE_SHORT      = 81,
    O_DOG              = 90, 
    O_POS_STEP         = 95, 
    O_SPEED_STEP       = 96, 
    O_MOS_FAIL         = 97, 
} MC_REE_P;

class DDJ_M15
{
public:
    DDJ_M15():rev(0), enc_pos(0),
        pos(0), vel(0), iq(0){} 

    double pos;
    double vel;
    double iq;

    int    rev;
    int16_t enc_pos;

    MC_REE_P error;
    uint8_t mode;

    /**
     * @brief update serial 
     * @note  NON-API FUNCTION
     */
    void update_serial(OSDK_Push_Data_M15_t& data);
};

class Leg_Motors{
public:
    DDJ_M15 left_hip;
    DDJ_M15 left_knee;
    DDJ_M15 left_wheel;
    DDJ_M15 right_hip;
    DDJ_M15 right_knee;
    DDJ_M15 right_wheel;
};
