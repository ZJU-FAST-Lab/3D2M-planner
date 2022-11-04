#include "OSDK_Header.hpp"
#include "OSDK_Vehicle.hpp"
#include "OSDK_Virtual_RC.hpp"

using namespace DIABLO::OSDK;

uint8_t Virtual_RC::obtain_control(uint16_t timeout_ms)
{
    if(this->in_control()) return 5;
    if(vehicle->telemetry->status.ctrl_mode & ((1<<(vehicle->telemetry->id*2)) | (1<<(4+vehicle->telemetry->id*2))))
        exit(0);        // control is taken by Movement Ctrl
    
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Virtual_RC_Request_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK     = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    OSDK_Virtual_RC_Request_t req;
    req.request     = 1;
    req.timeout_act = 0;
    req.timeout_ms  = timeout_ms;

    uint16_t ack = -1;
    while(ack != 0x0002)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
        OSDK_VIRTUAL_RC_SET, OSDK_VIRTUAL_RC_AUTHORIZE_ID, 
        &req, sizeof(OSDK_Virtual_RC_Request_t));

        if(result) return result;
        if(ack == 0x0000)
        {
            std::cerr<<"SDK control disabled on manual RC controller, check your RC setting"<<std::endl;
            return 3;
        }
        if(ack == 0x000A)
        {
            std::cerr<<"Cannot switch to SDK control, check robot status"<<std::endl;
            return 3;
        }
        usleep(5000);
    }

    this->ctrl_status = CTRL_OBTAINED;
    idle_buffer = true;
    std::cout<<"SDK Virtual RC In Control"<<std::endl;
    //abort();
    return 0;
}

uint8_t Virtual_RC::release_control()
{
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Virtual_RC_Request_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK     = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    OSDK_Virtual_RC_Request_t req;
    req.request     = 0;
    req.timeout_act = req.timeout_ms = 0;

    vehicle->hal->serialSend(header.data, 
        OSDK_VIRTUAL_RC_SET, OSDK_VIRTUAL_RC_AUTHORIZE_ID, 
        &req, sizeof(OSDK_Virtual_RC_Request_t));

    this->ctrl_status = CTRL_RELEASED;
    this->vehicle->telemetry->connection_check();
    return 0;
}

uint8_t Virtual_RC::SendVirtualRCCmd()
{
    //printf("send VRC Cmd\n");
    if(this->ctrl_status == CTRL_IDLE)
    {
        if(idle_buffer)
        {
            printf("Another SDK takes the control. Current SDK is set to idle\n");
            this->vehicle->telemetry->connection_check();
            idle_buffer = false;
        }
        return 0;
    }
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Virtual_RC_t) + OSDK_MISC_SIZE;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    return vehicle->hal->serialSend(header.data, 
        OSDK_VIRTUAL_RC_SET, OSDK_VIRTUAL_RC_DATA_ID, 
        &data, sizeof(OSDK_Virtual_RC_t));
}

void Virtual_RC::CtrlStatusMonitorHandle(const uint8_t ctrl_mode)
{
    if(ctrl_mode & (1<<(1 + vehicle->telemetry->id*2)))
    {
        ctrl_status = CTRL_OBTAINED;
        dropCtrlCnt = 0;
    }
    else if(ctrl_mode & (1<<(5 + vehicle->telemetry->id*2)))
    {
        if(ctrl_mode & 0x0F)
            ctrl_status = CTRL_IDLE;
        else
            ctrl_status = CTRL_RELEASED;
        dropCtrlCnt = 0;
    }
    else        //if(ctrl_mode != OSDK_CTRL_MODE_VIRTUAL_RC)
    {
        if(dropCtrlCnt > 10)
        {
            if(ctrl_status == CTRL_OBTAINED)
                std::cout<<"Virtual RC control authority released by vehicle"<<std::endl;
            this->SerialDisconnectHandle();
        }
        else
            dropCtrlCnt++;
    }
}
