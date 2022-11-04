
#include "OSDK_HAL.hpp"
#include "OSDK_Header.hpp"
#include "OSDK_CRC.hpp"


using namespace DIABLO::OSDK;
using namespace boost;
using namespace boost::chrono;

void HAL::serialPackData(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len)
{
    memcpy(serial_txbuf, &header, sizeof(OSDK_Uart_Header_t));
    serial_txbuf[sizeof(OSDK_Uart_Header_t)]     = cmd_set;
    serial_txbuf[sizeof(OSDK_Uart_Header_t) + 1] = cmd_id;
    memcpy(serial_txbuf + OSDK_DATA_POS_OFFSET, data, data_len);
    uint16_t CRC16 = DIABLO::Utility::update_crc16(serial_txbuf, OSDK_DATA_POS_OFFSET + data_len);
    memcpy(serial_txbuf + OSDK_DATA_POS_OFFSET + data_len, &CRC16, 2);
}

void* HAL::serialWaitRXDataS(boost::unique_lock<boost::mutex>& lock, const uint8_t cmd_set, const uint8_t cmd_id)
{
    static const chrono::duration<int, milli> timeout(1000);
    if(!serial_rx_data_cond.wait_for(lock, timeout,
        std::bind(&DIABLO::OSDK::HAL::verifyRXType, this, cmd_set, cmd_id))) return NULL;

    *((uint8_t*)rx_data - 2) = *((uint8_t*)rx_data - 1) = -1; //mark rx cmd set and id to be invalid
    return getRXData();
}

uint8_t HAL::serialSend_ack(const OSDK_Uart_Header_t& header, uint16_t& ack,
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len)
{
    if(serialSend(header, cmd_set, cmd_id, data, data_len)) 
    {
        printf("no ack\n");
        return 1; //transmit timeout
    }

    static const chrono::duration<int, milli> timeout(100);
    {
        //printf("ack\n");
        unique_lock<mutex> lock(serial_rx_mtx);
        if(!serial_rx_ack_cond.wait_for(lock, timeout, 
            std::bind(&DIABLO::OSDK::HAL::verifyRXType, this, cmd_set, cmd_id))) 
        {
            printf("wait ack timeout\n");
            return 2; //wait ack timeout
        }
        ack = this->getACK();
    }


    return 0;
}

void HAL::TXMonitorProcess(void)
{
    static const chrono::duration<int, milli> timeout(100);
    while(true)
    {
        {
            unique_lock<mutex> lock(serial_tx_mtx);
            if(serial_tx_idle && 
              !serial_tx_cond.wait_for(lock, timeout, [this](void){return !serial_tx_idle;}))
            {
                serial_tx_idle = true;
                continue;
            }
        }
        this_thread::sleep_for(boost::chrono::duration<double>(serial_tx_duration));
        {
            unique_lock<mutex> lock(serial_tx_mtx);
            serial_tx_idle = true;
            serial_tx_cond.notify_all();
        }
    }
}


uint8_t HAL_Pi::serialSend(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len)
{
    static const chrono::duration<int, milli> timeout(100);
    unique_lock<mutex> lock(serial_tx_mtx);

    if(!serial_tx_idle && 
       !serial_tx_cond.wait_for(lock, timeout, [this](void){return serial_tx_idle;}))
        return 1;
    
    serialPackData(header, cmd_set, cmd_id, data, data_len);

    size_t size = OSDK_DATA_POS_OFFSET+data_len+2;

    serial_tx_duration = size*10./serial_br + 2e-4;
    serial_seq++;
    serial_tx_idle = false;
    serial_tx_cond.notify_all();

    for(uint8_t i = 0; i < size; i++) 
        mySerial.WriteChar(serial_txbuf[i]);
    
    return 0;
}

void HAL_Pi::RXMonitorProcess(void)
{
    Header header;
    uint8_t* p_header = (uint8_t*)(&header);
    uint32_t byte_micro = 1000000 * 10/serial_br;
    while(true)
    {
        int start = -1;
        while(start != OSDK_HEADER)
        {
            start = mySerial.ReadChar();
            if(start == -1) //no data received at all
                std::cerr<<"Serial receive timeout occured 1!"<<std::endl;
        }
        
        //wait for receiving a complete header
        usleep(byte_micro * sizeof(OSDK_Uart_Header_t));
        while(mySerial.Available() < sizeof(OSDK_Uart_Header_t) - 1) usleep(byte_micro);
        
        header.data.SOF = start;
        uint16_t len, data_len;
        for(uint16_t i = 1; i < sizeof(OSDK_Uart_Header_t); i++)
        {
            int result = mySerial.ReadChar();
            if(result == -1)
            {
                std::cerr<<"Serial receive timeout occured 2!"<<std::endl;
                header.data.SOF = 0xFF;
		        continue;
            }
            p_header[i] = result;
        }

        len = header.data.LEN;
        if(
            !header.verify() || //bad header
             len > 255 //invalid length
          )
        {
            header.data.SOF = 0xFF;
            continue;
        }
        data_len = len - sizeof(OSDK_Uart_Header_t);

        //wait for receiving a complete frame
        usleep(byte_micro * data_len);
        while(mySerial.Available() < data_len) usleep(byte_micro);

        unique_lock<mutex> lock(serial_rx_mtx);
        for(uint16_t i = sizeof(OSDK_Uart_Header_t); i < len; i++)
        {
            int result = mySerial.ReadChar();
            
            if(result == -1)
            {
                std::cerr<<"Serial receive timeout occured 3!"<<std::endl;
                header.data.SOF = 0xFF;
                lock.unlock();
                continue;
            }
            serial_rxbuf[i] = result;
        }

        memcpy(serial_rxbuf, &header, sizeof(OSDK_Uart_Header_t));
        if(!DIABLO::Utility::verify_crc16(serial_rxbuf, len))
        {
            header.data.SOF = 0xFF;
            lock.unlock();
            continue;
        }
        
        if(header.data.ACK)    serial_rx_ack_cond.notify_all();
        else                   serial_rx_data_cond.notify_all();

        header.data.SOF = 0xFF; //mark the start byte to be invalid
        lock.unlock();
    }
}



