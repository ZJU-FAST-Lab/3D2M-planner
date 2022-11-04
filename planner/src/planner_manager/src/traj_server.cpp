#include <ros/ros.h>
#include "mpc/Polynome.h"
#include "planner_algorithm/poly_traj_utils.hpp"
#include "planner_algorithm/back_end_optimizer.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
using namespace Eigen;

ros::Publisher despoint_vis_pub;
ros::Publisher despoint_pub;
ros::Publisher control_cmd_pub;

double dt = 0.1;
double t_cur;

bool has_traj = false;
bool has_odom = false;

Trajectory trajectory;
minco::MinJerkOpt jerkOpt_;
double traj_duration;
ros::Time start_time;
int traj_id;


//odometry on real time
Eigen::Vector3d  odometry_pos;
Eigen::Vector3d  odometry_vel;
double           odometry_yaw;

void tempRenderAPoint(Vector3d pt, Vector3d color)
{
    visualization_msgs::Marker sphere;
    sphere.header.frame_id      = "world";
    sphere.header.stamp         = ros::Time::now();
    sphere.type                 = visualization_msgs::Marker::SPHERE;
    sphere.action               = visualization_msgs::Marker::ADD;
    sphere.id                   = 1;
    sphere.pose.orientation.w   = 1.0;
    sphere.color.r              = color(0);
    sphere.color.g              = color(1);
    sphere.color.b              = color(2);
    sphere.color.a              = 0.8;
    sphere.scale.x              = 0.2;
    sphere.scale.y              = 0.2;
    sphere.scale.z              = 0.2;
    sphere.pose.position.x      = pt(0);
    sphere.pose.position.y      = pt(1);
    sphere.pose.position.z      = pt(2);
    despoint_vis_pub.publish(sphere);
}

void rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
  if(has_odom == false){ cout <<"[TRAJ_SERVER] has odometry "<<endl; }
  has_odom = true;
  odometry_pos[0] = msg->pose.pose.position.x;
  odometry_pos[1] = msg->pose.pose.position.y;
  odometry_pos[2] = msg->pose.pose.position.z;

  
  odometry_vel[0] = msg->twist.twist.linear.x;
  odometry_vel[1] = msg->twist.twist.linear.y;
  odometry_vel[2] = msg->twist.twist.linear.z;

  Eigen::Quaterniond q( msg->pose.pose.orientation.w,
			                  msg->pose.pose.orientation.x,
		                  	msg->pose.pose.orientation.y,
		                  	msg->pose.pose.orientation.z );
  Eigen::Matrix3d R(q);
  odometry_yaw = atan2(R.col(0)[1],R.col(0)[0]);
  
}

void polynomialTrajCallback(mpc::PolynomeConstPtr msg)
{
  // parse pos traj
  MatrixXd posP(3, msg -> pos_pts.size() - 2);
  VectorXd T(msg -> t_pts.size());
  MatrixXd initS, tailS;

  for (int i = 1; i < msg -> pos_pts.size() - 1 ;i++)
  {
    posP(0, i-1) = msg->pos_pts[i].x;
    posP(1, i-1) = msg->pos_pts[i].y;
    posP(2, i-1) = msg->pos_pts[i].z;
  }
  for (int i=0; i<msg->t_pts.size();i++)
  {
    T(i) = msg->t_pts[i];
  }
  
  initS.setZero(3, 3);
  tailS.setZero(3, 3);
  initS.col(0) = Vector3d(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
  initS.col(1) = Vector3d(msg->init_v.x, msg->init_v.y, msg->init_v.z);
  initS.col(2) = Vector3d(msg->init_a.x, msg->init_a.y, msg->init_a.z);
  tailS.col(0) = Vector3d(msg->pos_pts.back().x, msg->pos_pts.back().y, msg->pos_pts.back().z);
  tailS.col(1) = Vector3d::Zero();
  tailS.col(2) = Vector3d::Zero();
  jerkOpt_.reset(initS, msg->pos_pts.size()-1);
  jerkOpt_.generate(posP, tailS, T);

  trajectory    = jerkOpt_.getTraj();
  traj_duration = trajectory.getTotalDuration();

  start_time  = msg -> start_time;

  if(has_traj == false){ cout <<"[TRAJ_SERVER] has trajectory "<<endl; }
  has_traj = true;
}

double err_yaw( double des_yaw, double odom_yaw)
{
  if(des_yaw - odom_yaw >= pi)
    return (des_yaw - odom_yaw) - 2 * pi;
  else if(des_yaw - odom_yaw <= -pi)
    return 2 * pi + (des_yaw - odom_yaw);
  else
    return (des_yaw - odom_yaw); 
}

void cmdCallback(const ros::TimerEvent &e)
{
    if ((!has_traj) || (!has_odom))
      return;
    
    ros::Time time_now  = ros::Time::now();
    t_cur               = (time_now - start_time).toSec();

    geometry_msgs::Pose cmd;

    if (t_cur < traj_duration && t_cur >= 0.0)
    {
      Vector3d des_pos   = trajectory.getPos(t_cur);
      Vector3d des_vel   = trajectory.getVel(t_cur);
      Vector3d des_acc   = trajectory.getAcc(t_cur);
      Vector2d des_velxy   = Vector2d(des_vel(0), des_vel(1));
      tempRenderAPoint(des_pos, Vector3d(0.1,0.2,0.9) );
    
      double des_yawdot  =  (des_acc(1) * des_vel(0) - des_acc(0) * des_vel(1)) / (des_velxy.squaredNorm());

      Vector3d err_pos   = des_pos - odometry_pos;

      double des_yaw     = atan2(err_pos(1), err_pos(0));
      Vector3d err_vel   = des_vel - odometry_vel;
      Vector2d err_posxy = Vector2d(err_pos(0), err_pos(1));
      Vector2d err_velxy = Vector2d(err_vel(0), err_vel(1));
      Vector2d forward_dir = Vector2d( cos(odometry_yaw) , sin(odometry_yaw) );

      double ef  = forward_dir.dot(err_posxy);
      double def = forward_dir.dot(err_velxy);

      double forward_speed = des_velxy.norm() + ( 5 * ef + 0.5 * def );
      double ori_speed     = des_yawdot + (4 * err_yaw(des_yaw , odometry_yaw) );
      double vertical_speed = des_vel(2) + (2 * (des_pos(2) - odometry_pos(2)));
      // if(ori_speed > 5)
      // {
      //   cout<<"dyd = " << des_yawdot <<" | dy = " << des_yaw << "  | odom_yaw = " << odometry_yaw <<endl;
      // }

      double L = 0.5;
      cmd.position.x  =  (forward_speed - L * ori_speed) / 2;
      cmd.position.y =  (forward_speed + L * ori_speed) / 2;
      cmd.position.z     =  vertical_speed;
      cmd.orientation.z  =  des_pos(2) - 0.7;

      control_cmd_pub.publish(cmd);
    }

    else if(t_cur >= traj_duration)
    {
      Vector3d des_pos   = trajectory.getPos(traj_duration);
      cmd.position.x  =  0;
      cmd.position.y  =  0;
      cmd.position.z  =  0;
      cmd.orientation.z  =  des_pos(2) - 0.7;

      control_cmd_pub.publish(cmd);
    }

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  ros::Subscriber traj_sub      = nh.subscribe("trajectory_topic", 10, polynomialTrajCallback);
  ros::Subscriber odom_sub      = nh.subscribe("odom", 1, rcvOdomCallBack );

  despoint_pub      = nh.advertise<geometry_msgs::PoseStamped>("despoint", 20); 
  despoint_vis_pub  = nh.advertise<visualization_msgs::Marker>("point/vis", 20); 
  control_cmd_pub   = nh.advertise<geometry_msgs::Pose>("controller_cmd", 20); 

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  ros::Duration(1.0).sleep();
  ROS_WARN("[Traj server]: ready.");
  ros::spin();

  return 0;
}