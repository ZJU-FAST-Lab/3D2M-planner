/**
 * @file OSDK_Header.hpp
 * @brief Header class store data of packet header and handle all issues with header
 */
#pragma once

#include <stdio.h>
#include <iostream>
#include <string.h>
#include "Onboard_SDK_Uart_Protocol.h"
#include "OSDK_CRC.hpp"

namespace DIABLO{
namespace OSDK{
struct Header{
public:
    Header()
    {
        memset(&data, 0, sizeof(OSDK_Uart_Header_t));
        data.SOF = OSDK_HEADER;
    }

    /**
     * @brief append crc16 for packet head
     * @note NON-API FUNCTION
     */
    void append_crc(void)
    {
        data.CRC16 = DIABLO::Utility::update_crc16(&this->data, 
            sizeof(OSDK_Uart_Header_t) - 2);
    }

    /**
     * @brief verify receiving packet's head crc16
     * 
     * @return true head data is correct
     * @return false head data has been disturbed
     */
    bool verify(void)
    {
        return this->data.SOF == OSDK_HEADER &&
               DIABLO::Utility::verify_crc16(&this->data, sizeof(OSDK_Uart_Header_t));
        //return this->data.SOF == OSDK_HEADER;
    }

public:
    OSDK_Uart_Header_t data;
};
}
}