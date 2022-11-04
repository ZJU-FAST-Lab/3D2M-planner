/**
 * @file OSDK_HAL.hpp
 * @brief SDK HAL Library, handle all communication issue
 */
#pragma once

#include <cstring>
#include <list>
#include <pthread.h>
#include <iostream>

#include "VulcanSerial/SerialPort.hpp"
#include "Onboard_SDK_Uart_Protocol.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/chrono.hpp>

namespace DIABLO{
namespace OSDK{

/**
 * @brief Abstract HAL class
 */
class HAL
{
public:
    /** 
     * @brief get transmission packed sequence number
     * @note  NON-API FUNCTION
     */
    uint32_t serial_getSeq(void)
    {
        return serial_seq;
    }

    /** 
     * @brief get data pointer to received pack 
     * @note  NON-API FUNCTION
     */
    void* getRXData(void)
    {
        return rx_data;
    }

    /** 
     * @brief get cmd set to received pack 
     * @note  NON-API FUNCTION
     */
    uint8_t getRXCmdSet(void)
    {
        return *((uint8_t*)rx_data - 2);
    }

    /** 
     * @brief get cmd id to received pack 
     * @note  NON-API FUNCTION
     */
    uint8_t getRXCmdID(void)
    {
        return *((uint8_t*)rx_data - 1);
    }

    /** 
     * @brief get ACK value
     * @note  NON-API FUNCTION
     */
    uint16_t getACK(void)
    {
        return *((uint16_t*)rx_data);
    }

    /**
     * @brief get baudrate
     * @note NON-API FUNCTION
     */
    uint32_t getSerialBr(void)
    {
        return serial_br;
    }

    /**
     * @brief format serial data
     * @note NON-API FUNCTION
     */
    void serialPackData(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len);

    /** 
     * @brief process serial transmission of generic SDK message in non-blocking mode
     * @note  NON-API FUNCTION
     * @return 0: successfully send \n
     *         1: multiplexer wait timeout(most possible reason is bandwidth full) \n
     */
    virtual uint8_t serialSend(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len) = 0;
    
    /** 
     * @brief process serial transmission of generic SDK message, and wait for ack packet
     * @note  NON-API FUNCTION
     * @return 0: successfully send \n
     *         1: multiplexer wait timeout(most possible reason is bandwidth full) \n
     *         2: ack packet wait timeout \n
     */
    uint8_t serialSend_ack(const OSDK_Uart_Header_t& header, uint16_t& ack,
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len);

    /**
     * @brief       serial wait receive data 
     * @param[in]   cmd_set cmd set value of packet to wait
     * @param[in]   cmd_id  cmd id value of packet to wait
     * @note        NON-API FUNCTION
     * @return      pointer to data packet, NULL if no packet received in 100ms
     */
    void* serialWaitRXDataS(boost::unique_lock<boost::mutex>& lock, 
        const uint8_t cmd_set, const uint8_t cmd_id);

    /**
     * @brief       get SDK time stamp
     * @return      a double-precision number indicates seconds elasped from the start of SDK 
     */
    double getTimeStamp(void)
    {
        boost::chrono::duration<double> sec = boost::chrono::system_clock::now() - start_tp;
        return sec.count();
    }

protected:
    /** 
     * @brief serial receive thread function
     * @note  NON-API FUNCTION
     */
    virtual void RXMonitorProcess(void) = 0;

    /**
     * @brief check rx type is data or ack
     * @note  NON-API FUNCTION
     */
    bool verifyRXType(const uint8_t cmd_set, const uint8_t cmd_id)
    {
        //printf("%u\t%u\n%u\t%u\n",cmd_id,cmd_set,this->getRXCmdID(),this->getRXCmdSet());
        return cmd_set == this->getRXCmdSet() && cmd_id == this->getRXCmdID();
    }

public:
    boost::mutex                     serial_rx_mtx;
    boost::mutex                     serial_tx_mtx;
    boost::mutex                  serial_prerx_mtx;

protected:
    HAL():
        serial_tx_thd(NULL), serial_rx_thd(NULL),
        start_tp(boost::chrono::system_clock::now()), VRC_Data_packet_num(0), VRC_REQ_packet_num(0)
        {}

    ~HAL()
    {
        if(serial_tx_thd)
        {
            pthread_cancel(serial_tx_thd->native_handle());
            delete serial_tx_thd;
            serial_tx_thd = NULL;
        }
        if(serial_rx_thd)
        {
            pthread_cancel(serial_rx_thd->native_handle());
            delete serial_rx_thd;
            serial_rx_thd = NULL;
        }
    }

    uint32_t                            serial_seq;
    uint32_t                             serial_br;
    uint8_t                      serial_txbuf[256];
    uint8_t                      serial_rxbuf[256];
    void*                                  rx_data;
    uint8_t                   serial_prerxbuf[256];
    uint8_t                               prebytes;
    bool                             first_rx_flag;

protected:
//transmission handling
    bool                            serial_tx_idle;
    boost::condition_variable       serial_tx_cond;
    double                      serial_tx_duration;
    boost::thread*                   serial_tx_thd;

//receive handling
    boost::thread*                   serial_rx_thd;
    
    boost::condition_variable   serial_rx_ack_cond;
    boost::condition_variable  serial_rx_data_cond;

    /** 
     * @brief serial transmission multiplexing thread function
     * @note  NON-API FUNCTION
     */
    void TXMonitorProcess(void);

protected:
    boost::chrono::system_clock::time_point start_tp;

protected:
    uint16_t VRC_Data_packet_num;
    uint16_t VRC_REQ_packet_num;
};


/**
 * @brief HAL class for Raspiberry Pi
 */
class HAL_Pi: public HAL
{
public:
    HAL_Pi(){}

    /**
     * @brief initialize serial port for raspberry pi
     * @param dev name of the port
     * @param baud serial baudrate
     * @return 0 initialization ok \n
     *         1 start wiringPi error \n
     *         2 start heartbeat error \n
     *         3 initialization port fail \n
     */
    // uint8_t init(const std::string dev = "/dev/ttyAMA0", const int baud = 460800)
    uint8_t init(const std::string dev = "/dev/ttyUSB0", const int baud = 460800)
    {
        mySerial.SetDevice(dev.c_str());
        mySerial.SetBaudRate(VulcanSerial::BaudRate::B_460800);
        mySerial.SetNumDataBits(VulcanSerial::NumDataBits::EIGHT);
        mySerial.SetNumStopBits(VulcanSerial::NumStopBits::ONE);
        serial_br = baud;
        mySerial.Open();
        
        serial_tx_duration = 0;
        serial_tx_idle = true;
        serial_tx_thd = new boost::thread(std::bind(&HAL_Pi::TXMonitorProcess, this));
        serial_rx_thd = new boost::thread(std::bind(&HAL_Pi::RXMonitorProcess, this)); //TODO CPU load too heavy, modify this
        
        usleep(10000); //ensure stability
        std::cout<<"Serial port \""<<dev<<"\" connected"<<std::endl;   

        rx_data = (void*)(serial_rxbuf + 2 + sizeof(OSDK_Uart_Header_t));
        return 0;
    }

    ~HAL_Pi()
    {
        mySerial.Close();
    }

    /**
     * @brief process serial transmission of generic SDK message in non-blocking mode
     * @note NON-API FUNCTION
     * @return 0: successfully send \n
     *         1: multiplexer wait timeout(most possible reason is bandwidth full) \n
     */
    uint8_t serialSend(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len);

    /** 
     * @brief serial receive thread function
     * @note  NON-API FUNCTION
     */
    void RXMonitorProcess(void);

private:
    VulcanSerial::SerialPort      mySerial;

};

}
}
