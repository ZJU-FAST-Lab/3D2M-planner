#include <ros/ros.h>
#include "diablo_sdk/OSDK_ACCL.h"
#include "diablo_sdk/OSDK_GYRO.h"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#include "diablo_sdk/OSDK_POWER.h"
#include "diablo_sdk/OSDK_QUATERNION.h"
#include "diablo_sdk/OSDK_RC.h"
#include "diablo_sdk/OSDK_STATUS.h"

#include "Onboard_SDK_Uart_Protocol.h"
#include "OSDK_Vehicle.hpp"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "robot_status_example");
    ros::NodeHandle nh("~");

    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init()) return -1;

    DIABLO::OSDK::Vehicle vehicle(&Hal);                                  //Initialize Onboard SDK
    if(vehicle.init()) return -1;

    vehicle.telemetry->activate();
    
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, OSDK_PUSH_DATA_10Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_100Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_50Hz);
    
    vehicle.telemetry->configUpdate(); 
    
//    vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_POWER);

    ros::Publisher ACCLPublisher = nh.advertise<diablo_sdk::OSDK_ACCL>("diablo_ros_ACCL_b", 10);
    ros::Publisher GYROPublisher = nh.advertise<diablo_sdk::OSDK_GYRO>("diablo_ros_GYRO_b", 10);
    ros::Publisher LEGMOTORSPublisher = nh.advertise<diablo_sdk::OSDK_LEGMOTORS>("diablo_ros_LEGMOTORS_b", 10);
    ros::Publisher POWERPublisher = nh.advertise<diablo_sdk::OSDK_POWER>("diablo_ros_POWER_b", 10);
    ros::Publisher QUATERNIONPublisher = nh.advertise<diablo_sdk::OSDK_QUATERNION>("diablo_ros_QUATERNION_b", 10);
    ros::Publisher RCPublisher = nh.advertise<diablo_sdk::OSDK_RC>("diablo_ros_RC_b", 10);
    ros::Publisher STATUSPublisher = nh.advertise<diablo_sdk::OSDK_STATUS>("diablo_ros_STATUS_b", 10);
    
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        if(vehicle.telemetry->newcome & 0x40)
        {
            diablo_sdk::OSDK_STATUS msg;
            msg.ctrl_mode = vehicle.telemetry->status.ctrl_mode;
            msg.robot_mode = vehicle.telemetry->status.robot_mode;
            msg.error = vehicle.telemetry->status.error;
            msg.warning = vehicle.telemetry->status.warning;
            STATUSPublisher.publish(msg);
            vehicle.telemetry->eraseNewcomeFlag(0xBF);
        }
        if(vehicle.telemetry->newcome & 0x20)
        {
            diablo_sdk::OSDK_QUATERNION msg;
            msg.w = vehicle.telemetry->quaternion.w;
            msg.x = vehicle.telemetry->quaternion.x;
            msg.y = vehicle.telemetry->quaternion.y;
            msg.z = vehicle.telemetry->quaternion.z;
            QUATERNIONPublisher.publish(msg);
            printf("Quaternion_w:\t%f\nQuaternion_x:\t%f\nQuaternion_y:\t%f\nQuaternion_z:\t%f\n", msg.w, msg.x, msg.y, msg.z);
            vehicle.telemetry->eraseNewcomeFlag(0xDF);
        }
        if(vehicle.telemetry->newcome & 0x10)
        {
            diablo_sdk::OSDK_ACCL msg;
            msg.x = vehicle.telemetry-> accl.x;
            msg.y = vehicle.telemetry-> accl.y;
            msg.z = vehicle.telemetry-> accl.z;
            ACCLPublisher.publish(msg);
            printf("ACCL_X:\t%f\nACCL_Y:\t%f\nACCL_Z:\t%f\n", msg.x, msg.y, msg.z);
            vehicle.telemetry->eraseNewcomeFlag(0xEF);
        }
        if(vehicle.telemetry->newcome & 0x08)
        {
            diablo_sdk::OSDK_GYRO msg;
            msg.x = vehicle.telemetry->gyro.x;
            msg.y = vehicle.telemetry->gyro.y;
            msg.z = vehicle.telemetry->gyro.z;
            GYROPublisher.publish(msg);
            printf("GYRO_X:\t%f\nGYRO_Y:\t%f\nGYRO_Z:\t%f\n", msg.x, msg.y, msg.z);
            vehicle.telemetry->eraseNewcomeFlag(0xF7);
        }
        if(vehicle.telemetry->newcome & 0x04)
        {
            diablo_sdk::OSDK_RC msg;
            msg.ch1 = vehicle.telemetry->rc.ch1;
            msg.ch2 = vehicle.telemetry->rc.ch2;
            msg.ch3 = vehicle.telemetry->rc.ch3;
            msg.ch4 = vehicle.telemetry->rc.ch4;
            msg.ch5 = vehicle.telemetry->rc.ch5;
            msg.ch6 = vehicle.telemetry->rc.ch6;
            msg.ch7 = vehicle.telemetry->rc.ch7;
            msg.ch8 = vehicle.telemetry->rc.ch8;
            msg.ch9 = vehicle.telemetry->rc.ch9;
            msg.ch10 = vehicle.telemetry->rc.ch10;
            msg.ch11 = vehicle.telemetry->rc.ch11;
            msg.ch12 = vehicle.telemetry->rc.ch12;
            msg.ch13 = vehicle.telemetry->rc.ch13;
            msg.ch14 = vehicle.telemetry->rc.ch14;
            msg.ch15 = vehicle.telemetry->rc.ch15;
            msg.ch16 = vehicle.telemetry->rc.ch16;
            msg.ch17 = vehicle.telemetry->rc.ch17;
            msg.ch18 = vehicle.telemetry->rc.ch18;
            msg.frame_lost = vehicle.telemetry->rc.frame_lost;
            msg.failsafe = vehicle.telemetry->rc.failsafe;
            msg.reserve = vehicle.telemetry->rc.reserve;
            RCPublisher.publish(msg);
            vehicle.telemetry->eraseNewcomeFlag(0xFB);
        }
        if(vehicle.telemetry->newcome & 0x02)
        {
            diablo_sdk::OSDK_POWER msg;
            msg.battery_voltage = vehicle.telemetry->power.voltage;
            msg.battery_current = vehicle.telemetry->power.current;
            msg.battery_capacitor_energy = vehicle.telemetry->power.capacitor_energy;
            msg.battery_power_percent = vehicle.telemetry->power.power_percent;
            POWERPublisher.publish(msg);
            printf("Power:\nVoltage:\t%f\nCurrent:\t%f\nCap_EN:\t%f\nPercent:\t%u\n", msg.battery_voltage, msg.battery_current, msg.battery_current, msg.battery_power_percent);
            vehicle.telemetry->eraseNewcomeFlag(0xFD);
        }
        if(vehicle.telemetry->newcome & 0x01)
        {
            printf("msg=%d\n",vehicle.telemetry->newcome);
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

            LEGMOTORSPublisher.publish(msg);
            vehicle.telemetry->eraseNewcomeFlag(0xFE);
        }
    }
    
    ros::spinOnce();
    loop_rate.sleep();
    return 0;
}
