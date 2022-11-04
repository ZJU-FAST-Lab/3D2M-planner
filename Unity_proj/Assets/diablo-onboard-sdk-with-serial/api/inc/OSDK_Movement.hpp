/**
 * @file OSDK_Movement.hpp
 * @brief Movement control class, to send movement control command to robot
 */
#pragma once

#include <stdio.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/chrono.hpp>
#include "Onboard_SDK_Uart_Protocol.h"

namespace DIABLO{
namespace OSDK{

// Forward Declaration
class HAL;
class Vehicle;

class Movement_Ctrl{
public:
    Movement_Ctrl(Vehicle* vehicle): vehicle(vehicle), ctrl_status(CTRL_RELEASED), dropCtrlCnt(0), idle_buffer(false)
    {
        //memset(&ctrl_mode_data, 0, sizeof(OSDK_Movement_Ctrl_Mode_t));
        memset(&ctrl_data, 0, sizeof(OSDK_Movement_Ctrl_t));
        ctrl_mode_cmd = false;
        ctrl_mode_data.head_controller_mode=0;
        ctrl_mode_data.height_ctrl_mode=0;
        ctrl_mode_data.pitch_ctrl_mode=0;
        ctrl_mode_data.split_ctrl_mode=0;
        ctrl_mode_data.roll_ctrl_mode=0;
        ctrl_mode_data.yaw_ctrl_mode=1;
        transform_data.transform_up = 0;
        transform_data.transform_down = 0;
        ctrl_data.up = 1.0f;
        //SendMovementModeCtrlCmd();
    }

    ~Movement_Ctrl()
    {
        if(this->release_control())
            std::cerr<<"Failed to release control Virtual RC"<<std::endl;
    }

    enum ctrl_status_t{
        CTRL_RELEASED = 0,
        CTRL_PENDING  = 1,
        CTRL_OBTAINED = 2,
        CTRL_IDLE     = 3
    };

    /**
     * @brief obtain control authority of the vehicle using motion control
     * @param[in] timeout_ms : automatically release control authority after not receiving valid message for some milliseconds
     * @note this function will automatically transmit two packets of "obtain control" frame. If control authorization is given to virtual RC. it will exit the program
     */
    uint8_t obtain_control(uint16_t timeout_ms = 500);

    /**
     * @brief   release control authority of the vehicle
     */
    uint8_t release_control();

    /**
     * @brief check whether vehicle is under control
     */
    bool in_control(void)
    {
        return (ctrl_status == CTRL_OBTAINED || ctrl_status == CTRL_IDLE);
    }

    /**
     * @brief handle serial disconnection event
     * @note  NON-API FUNCTION
     */
    void SerialDisconnectHandle()
    {
        this->ctrl_status = CTRL_RELEASED;
    }

    /**
     * @brief monitor control status and switch off control in case of disconnection
     * @note  NON-API FUNCTION
     */
    void CtrlStatusMonitorHandle(const uint8_t ctrl_mode);

    /**
     * @brief   Set emergency brake mode of robot
     * @note    The emergency brake will shut down all leg motors and brake the wheels,
     *          BEWARE OF POTENTIAL DAMAGE!
     * 
     *          The emergency brake mode can be released by two ways:
     *          re-power the robot or call EmergencyBrakeRelease()
     */
    uint8_t EmergencyBrake(void);
    
    /**
     * @brief   Release emergency brake mode of robot
     */
    uint8_t EmergencyBrakeRelease(void);

    /**
     * @brief Set movement control mode of robot
     * 
     * @return 0: Set control mode successfully \n
     *         other: See serialSend_ack error code \n
     */
    uint8_t SendMovementModeCtrlCmd();

    /**
     * @brief send movement command
     * 
     * @return 0: Set control mode successfully \n
     *         other: See serialSend_ack error code \n
     */
    uint8_t SendMovementCtrlCmd();

    /**
     * @brief send Transform up command (car to balance)
     * 
     * @return 0: Transform up successfully \n
     *         0xFF: Transform fail \n
     */
    uint8_t SendTransformUpCmd();

    /**
     * @brief send Transform down command (balance to car)
     * 
     * @return 0: Transform down successfully \n
     *         0xFF: Transform fail \n
     */
    uint8_t SendTransformDownCmd();


public:
    OSDK_Movement_Ctrl_Mode_t   ctrl_mode_data;
    OSDK_Movement_Ctrl_t             ctrl_data;

    bool                         ctrl_mode_cmd;
private:
    Vehicle*                           vehicle;
    ctrl_status_t                  ctrl_status;
    OSDK_Transform_Cmd_t        transform_data;
    uint8_t                        dropCtrlCnt;

    bool                           idle_buffer;
};

};
};
