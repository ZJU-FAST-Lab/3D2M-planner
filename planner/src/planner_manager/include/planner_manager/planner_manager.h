#ifndef PLANNER_MANAGER_H
#define PLANNER_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "map_manager/PCSmap_manager.h"
#include "planner_algorithm/poly_traj_utils.hpp"
#include "planner_algorithm/front_end_Astar.h"
#include "planner_algorithm/back_end_optimizer.h"


class PlannerManager
{
  public:
    PlannerManager(){};
    ~PlannerManager(){};
    void init(ros::NodeHandle& nh);

    void mapRcvCallBack(const std_msgs::Empty& msg);
    void targetRcvCallBack(const geometry_msgs::PoseStamped& msg);
    void target2RcvCallBack(const geometry_msgs::PoseStamped& msg);

    void rcvBenchmarkRun(const std_msgs::Int16 msg);
    void odomRcvCallBack(const nav_msgs::Odometry& msg);

    // point correction, for ground vehicle

    //void dropPoint(Vector3d &point);

    // vis
    void renderPath( vector<Vector3d> path );
    double renderTraj( Trajectory traj);
    void renderPoints(vector<Eigen::Vector3d> pts, Eigen::Vector3d color , double scale, int id);

    // planner
    bool generatePath( Vector3d start, Vector3d end );
    void generateTraj( vector<Vector3d> path , Vector3d start_vel, Vector3d target_vel, int& ret_value);

    void publishTraj();

    void pubFlash(Vector3d pos, Vector3d vel);
    void pubStop();


  private:

    // params
    double traj_parlength;
    double truncation_dis;

    //odom
    nav_msgs::Odometry recent_odom;
    bool has_odom;

    // planning
    vector<Vector3d> recent_path;
    Trajectory recent_traj;

    // ros
    ros::Subscriber target_sub2;
    ros::Subscriber target_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber rcvmap_signal_sub;
    ros::Subscriber benchmark_run_sub;

    ros::Publisher path_vis_pub;
    ros::Publisher traj_vis_pub;
    ros::Publisher point_vis_pub;

    ros::Publisher traj_pub;
    ros::Publisher flash_pub;
    ros::Publisher cmd_geo_pub;

  private:
    PCSmapManager::Ptr pcsmap_manager;
    AstarPathSearcher::Ptr astar_searcher;
    TrajOpt::Ptr minco_traj_optimizer;
};

#endif