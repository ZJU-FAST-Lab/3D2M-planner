#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>

#include "OSDK_Vehicle.hpp"
#include "diablo_sdk/Diablo_Ctrl.h"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>

#define head_control_mode 1
#define height_control_mode 1
#define pitch_control_mode 1

DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;
DIABLO::OSDK::HAL_Pi Hal;
DIABLO::OSDK::Vehicle vehicle(&Hal);
ros::Publisher LEGMOTORSPublisher;

void leg_callback(const ros::TimerEvent &e)
{
    if(vehicle.telemetry->newcome & 0x40)
    {
        vehicle.telemetry->eraseNewcomeFlag(0xBF);
    }
    if(vehicle.telemetry->newcome & 0x20)
    {
        vehicle.telemetry->eraseNewcomeFlag(0xDF);
    }
    if(vehicle.telemetry->newcome & 0x10)
    {
       vehicle.telemetry->eraseNewcomeFlag(0xEF);
    }
    if(vehicle.telemetry->newcome & 0x08)
    {
       vehicle.telemetry->eraseNewcomeFlag(0xF7);
    }
    if(vehicle.telemetry->newcome & 0x04)
    {
        vehicle.telemetry->eraseNewcomeFlag(0xFB);
    }
    if(vehicle.telemetry->newcome & 0x02)
    {
        vehicle.telemetry->eraseNewcomeFlag(0xFD);
    }

    if(vehicle.telemetry->newcome & 0x01)
    {
        diablo_sdk::OSDK_LEGMOTORS msg;
        msg.left_hip_enc_rev = vehicle.telemetry->motors.left_hip.rev;
        msg.left_hip_pos = vehicle.telemetry->motors.left_hip.pos;
        msg.left_hip_vel = vehicle.telemetry->motors.left_hip.vel;
        msg.left_hip_iq = vehicle.telemetry->motors.left_hip.iq;

        msg.left_knee_enc_rev = vehicle.telemetry->motors.left_knee.rev;
        msg.left_knee_pos = vehicle.telemetry->motors.left_knee.pos;
        msg.left_knee_vel = vehicle.telemetry->motors.left_knee.vel;
        msg.left_knee_iq = vehicle.telemetry->motors.left_knee.iq;

        msg.left_wheel_enc_rev = vehicle.telemetry->motors.left_wheel.rev;
        msg.left_wheel_pos = vehicle.telemetry->motors.left_wheel.pos;
        msg.left_wheel_vel = vehicle.telemetry->motors.left_wheel.vel;
        msg.left_wheel_iq = vehicle.telemetry->motors.left_wheel.iq;

        msg.right_hip_enc_rev = vehicle.telemetry->motors.right_hip.rev;
        msg.right_hip_pos = vehicle.telemetry->motors.right_hip.pos;
        msg.right_hip_vel = vehicle.telemetry->motors.right_hip.vel;
        msg.right_hip_iq = vehicle.telemetry->motors.right_hip.iq;

        msg.right_knee_enc_rev = vehicle.telemetry->motors.right_knee.rev;
        msg.right_knee_pos = vehicle.telemetry->motors.right_knee.pos;
        msg.right_knee_vel = vehicle.telemetry->motors.right_knee.vel;
        msg.right_knee_iq =  vehicle.telemetry->motors.right_knee.iq;

        msg.right_wheel_enc_rev = vehicle.telemetry->motors.right_wheel.rev;
        msg.right_wheel_pos = vehicle.telemetry->motors.right_wheel.pos;
        msg.right_wheel_vel = vehicle.telemetry->motors.right_wheel.vel;
        msg.right_wheel_iq = vehicle.telemetry->motors.right_wheel.iq;
    // msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = "world";
        LEGMOTORSPublisher.publish(msg);
        // printf("v = %lf     ", (msg.right_wheel_vel + msg.left_wheel_vel) * 0.935 / 2.0);
        // printf("w = %lf\n", (msg.right_wheel_vel - msg.left_wheel_vel) * 0.935 / 0.5);
        vehicle.telemetry->eraseNewcomeFlag(0xFE);
    }
}

void teleop_ctrl(const diablo_sdk::Diablo_CtrlConstPtr& msg)
{
    if(!pMovementCtrl->in_control())
    {
        printf("to get movement ctrl.\n");
        uint8_t result = pMovementCtrl->obtain_control();
        return;
    }

    
    if (fabs(msg->speed) > 1.5)
    {
        pMovementCtrl->ctrl_data.forward = msg->speed>0?1.5:-1.5;
    }
    else
    {
        pMovementCtrl->ctrl_data.forward = msg->speed;
    }

    if (fabs(msg->omega) > 2.4)
    {
        pMovementCtrl->ctrl_data.left = msg->omega>0?2.4:-2.4;
    }
    else
    {
        pMovementCtrl->ctrl_data.left = msg->omega;
    }

    if (fabs(msg->roll) > 0.15)
    {
        pMovementCtrl->ctrl_data.roll = msg->roll>0?0.15:-0.15;
    }
    else
    {
        pMovementCtrl->ctrl_data.roll = msg->roll;
    }

    if (pitch_control_mode == 1)
    {
        if (fabs(msg->pitch) > 0.9424778)
        {
            pMovementCtrl->ctrl_data.pitch = msg->pitch>0?0.9424778:-0.9424778;
        }
        else
        {
            pMovementCtrl->ctrl_data.pitch = msg->pitch;
        }
    }
    else
    {
        if (fabs(msg->pitch_vel) > 1.5)
        {
            pMovementCtrl->ctrl_data.pitch = msg->pitch_vel>0?1.5:-1.5;
        }
        else
        {
            pMovementCtrl->ctrl_data.pitch = msg->pitch_vel;
        }
    }

    if (height_control_mode == 1)
    {
        if (msg->height > 1.0)
        {
            pMovementCtrl->ctrl_data.up = 1.0;
        }
        else if (msg->height < 0.0)
        {
            pMovementCtrl->ctrl_data.up = 0.0;
        }
        else
        {
            pMovementCtrl->ctrl_data.up = msg->height;
        }
    }
    else
    {
        if (msg->height_vel > 0.2)
        {
            pMovementCtrl->ctrl_data.up = msg->height_vel>0?0.2:-0.2;
        }
        else
        {
            pMovementCtrl->ctrl_data.up = msg->height_vel;
        }
    }

    pMovementCtrl->SendMovementCtrlCmd();

    return;
}

void init_control_mode()
{
    if(!pMovementCtrl->in_control())
    {
        printf("to get movement ctrl.\n");
        uint8_t result = pMovementCtrl->obtain_control();
        return;
    }

    pMovementCtrl->ctrl_mode_cmd = true;
    // head control mode: 0: auto move to keep balance, 1: keep angle with ground
    pMovementCtrl->ctrl_mode_data.head_controller_mode = head_control_mode;
    // height control mode: 0: speed command, 1: position command
    pMovementCtrl->ctrl_mode_data.height_ctrl_mode = height_control_mode;
    // pitch control mode: 0: speed command, 1: position command
    pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = pitch_control_mode;

    pMovementCtrl->SendMovementModeCtrlCmd();
    
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diablo_ctrl");
    ros::NodeHandle nh("~");

    // DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init()) return -1;
    
    // DIABLO::OSDK::Vehicle vehicle(&Hal);                                  //Initialize Onboard SDK
    if(vehicle.init()) return -1;

    printf("%d\n",sizeof(OSDK_Uart_Header_t));

    vehicle.telemetry->activate();
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_50Hz);
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_100Hz);
    vehicle.telemetry->configUpdate();

    pMovementCtrl = vehicle.movement_ctrl;
    init_control_mode();
    ros::Subscriber sub = nh.subscribe("/diablo_cmd", 1, teleop_ctrl); //subscribe to ROS topic
    LEGMOTORSPublisher = nh.advertise<diablo_sdk::OSDK_LEGMOTORS>("diablo_leg_motors", 10);
    ros::Timer leg_timer = nh.createTimer(ros::Duration(0.001), &leg_callback);

    ros::spin();

    return 0;
}