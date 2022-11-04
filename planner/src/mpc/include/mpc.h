#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string.h>
#include <algorithm>
 
#include <OsqpEigen/OsqpEigen.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "cubic_spline_planner.h"
#include "diablo_sdk/Diablo_Ctrl.h"
#include "diablo_sdk/OSDK_LEGMOTORS.h"

#include "mpc_utils/traj_anal.hpp"

using namespace std;
using namespace Eigen;

class MPCState
{
public:
    double x = 0;
    double y = 0;
    double z = 0;
    double v = 0; 
    double theta = 0;
};

class MPC
{
public:
    enum TrajClass
    {
        MINCO,
        TVOTE,
        DRIPC
    };

private:
    // parameters
    /// algorithm param
    double du_th = 0.1;
    double dt = 0.2;
    int T = 5;
    int delay_num;
    int max_iter = 3;
    vector<double> Q = {10, 10, 2.5, 0.5};
    vector<double> R = {0.01, 0.01};
    vector<double> Rd = {0.01, 1.0};
    /// constraints
    double max_omega = M_PI / 4;
    double max_domega = M_PI / 6;
    double max_comega = M_PI / 6 * 0.2;
    double max_speed = 55.0 / 3.6;
    double max_cv = 0.2;
    double max_accel = 1.0; 

    // MPC dataset
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd C;
    MPCState xbar[500];
    Eigen::MatrixXd xref;
    Eigen::MatrixXd dref;
    Eigen::MatrixXd output;
    Eigen::MatrixXd last_output;
    std::vector<Eigen::Vector2d> output_buff;

    // control data
    bool has_odom;
    bool receive_traj_ = false;
    bool control_a = true;
    double tolerance = 0.1;
    double traj_duration_;
    double t_track = 0.0;
    ros::Time start_time_;
    MPCState now_state;
    TrajAnalyzer traj_analyzer;

    // ros interface
	ros::NodeHandle node_;
    ros::Timer cmd_timer_;
    ros::Publisher pos_cmd_pub_, pos_cmd_pub_geo_, vis_pub, predict_pub, ref_pub, init_pub, cmd_geo_pub, err_pub;
    ros::Subscriber odom_sub_, traj_sub_, trigger_sub_, vel_sub_, tv_sub_, cloud_sub_;

    // ros::Publisher pos_cmd_pub_, vis_pub, predict_pub, ref_pub;
    // ros::Subscriber odom_sub_, traj_sub_, trigger_sub_, vel_sub_, tv_sub_;
    diablo_sdk::Diablo_Ctrl cmd;
    void cmdCallback(const ros::TimerEvent &e);
    void rcvOdomCallBack(nav_msgs::OdometryPtr msg);
    void rcvVelCallBack(diablo_sdk::OSDK_LEGMOTORSConstPtr msg);
    void rcvTrajCallBack(mpc::PolynomeConstPtr msg);
    void rcvTriggerCallBack(const geometry_msgs::PoseStamped msg);
    void rcvTVPathCallBack(nav_msgs::PathConstPtr msg);
    void rcvCloudCallBack(sensor_msgs::PointCloud2ConstPtr msg);

    // for test tracking performance
    bool in_test;
    bool in_mocap;
    bool has_cloud;
    bool in_bk;
    TrajClass traj_class;
    cubic_spline_planner csp;
    string test_traj;
    std::ofstream outfile;
    vector<Eigen::Vector3d> csp_path;
    vector<Eigen::Vector3d> csp_path_3d;
    double tv_dist;

    // MPC function
    void getLinearModel(const MPCState& s);
    void stateTrans(MPCState& s, double a, double yaw_dot);
    void predictMotion(void);
    void predictMotion(MPCState *b);
    void solveMPCV(void);
    void solveMPCA(void);
    void getCmd(void);

    // utils
    MPCState xopt[500];
    void normlize_theta(double& th)
    {
        while (th > M_PI)
            th -= M_PI * 2;
        while (th < -M_PI)
            th += M_PI * 2;
    }
    void smooth_yaw(void)
    {
        double dyaw = xref(3, 0) - now_state.theta;

        while (dyaw >= M_PI / 2)
        {
            xref(3, 0) -= M_PI * 2;
            dyaw = xref(3, 0) - now_state.theta;
        }
        while (dyaw <= -M_PI / 2)
        {
            xref(3, 0) += M_PI * 2;
            dyaw = xref(3, 0) - now_state.theta;
        }

        for (int i=0; i<T-1; i++)
        {
            dyaw = xref(3, i+1) - xref(3, i);
            while (dyaw >= M_PI / 2)
            {
                xref(3, i+1) -= M_PI * 2;
                dyaw = xref(3, i+1) - xref(3, i);
            }
            while (dyaw <= -M_PI / 2)
            {
                xref(3, i+1) += M_PI * 2;
                dyaw = xref(3, i+1) - xref(3, i);
            }
        }
    }
    double get_nearest(void)
    {
        Eigen::Vector3d track_point = csp.get_state(t_track);
        Eigen::Vector2d now_point(now_state.x, now_state.y);
        double min_dist = (now_point-track_point.head(2)).norm();
        double track_temp = t_track;
        double interval = now_state.v * dt;
        
        for (double t_temp = track_temp + interval, i=1.0; i < 10; i += 1.0, t_temp += interval)
        // for (double t_temp = track_temp + 1.0, i=1.0; i < 10; i += 1.0, t_temp += 1.0)
        {
            if (t_temp > traj_duration_)
            {
                t_temp = traj_duration_;
            }
            Eigen::Vector3d temp = csp.get_state(t_temp);
            double dist = (temp.head(2) - now_point).norm();
         
            if (dist<min_dist)
            {
                min_dist = dist;
                t_track = t_temp;
            }
        }
        // ROS_INFO("t_track=%f,traj_duration=%f",t_track,traj_duration_);

        return t_track;
    }
    void drawFollowPath(void)
    {
        int id = 0;
        double scale = 1.0;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 1;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        line_strip.scale.x = scale / 2;
        geometry_msgs::Point pt;
        
        for (auto p:csp_path_3d)
        {
            pt.x = p(0);
            pt.y = p(1);
            pt.z = p(2);
            line_strip.points.push_back(pt);
        }
        vis_pub.publish(line_strip);
    }
    void drawPredictPath(MPCState *b)
    {
        int id = 0;
        double sc = 1.0;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 1;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::Point pt;
        
        for (int i=0; i<T; i++)
        {
            pt.x = b[i].x;
            pt.y = b[i].y;
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        predict_pub.publish(line_strip);
    }
    void drawRefPath(void)
    {
        int id = 0;
        double sc = 1.0;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::Point pt;
        
        for (int i=0; i<T; i++)
        {
            pt.x = xref(0, i);
            pt.y = xref(1, i);
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        ref_pub.publish(line_strip);
    }
    void drawBkPath(void)
    {
        int id = 0;
        double sc = 1.0;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::Point pt;
        
        std::vector<TrajPoint> now_traj = traj_analyzer.vis_trajs[traj_analyzer.track_seg-1];
        for (int i=0; i<now_traj.size(); i++)
        {
            pt.x = now_traj[i].x;
            pt.y = now_traj[i].y;
            pt.z = now_traj[i].z;
            line_strip.points.push_back(pt);
        }
        ref_pub.publish(line_strip);
    }

public:
	MPC() : csp(vector<double>{0.0, 0.0, 1.0, 1.5, 2.0, 3.0, 3.0, 2.0, 1.5, 1.0, 0.0},vector<double>{0.0, 1.5, 1.35, 0.75, 0.15, 0, 1.5, 1.35, 0.75, 0.15, 0.0},  0.1) {}    
    void init(ros::NodeHandle &nh);
	~MPC() {}
};