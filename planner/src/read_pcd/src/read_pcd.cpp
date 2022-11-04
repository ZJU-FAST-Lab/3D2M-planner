#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace Eigen;
// using namespace cv;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Publisher click_map_pub_;
ros::Subscriber _odom_sub;

vector<double> _state;
PointCloud<PointXYZRGB> ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
std::string mapFile;

bool _map_ok = false;
bool _has_odom = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;
uniform_real_distribution<double> rand_ground_height;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloudMap_filter;

sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

void RandomMapGenerate() {
  std::cout << "randomapgenerate "<<std::endl;
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZ>(mapFile,*cloudMap);

  
  cloudMap->width = cloudMap->points.size();
  cloudMap->height = 1;
  cloudMap->is_dense = true;
  double tmp=0;
  cout << "cloudMap: " << cloudMap->points.size() << endl;
 for(size_t i=0;i<cloudMap->points.size();i++)
 {
    pcl::PointXYZ pt_;
    pt_.x = cloudMap->points[i].x;
    pt_.y = cloudMap->points[i].y;
    pt_.z = cloudMap->points[i].z;
    cloudMap_filter.points.push_back(pt_);
 } 
  cloudMap_filter.width = cloudMap_filter.points.size();
  cloudMap_filter.height = 1;
  cloudMap_filter.is_dense = true;
  cout << "cloudMap_filter: " << cloudMap_filter.points.size() << endl;
  ROS_WARN("Finished generate random map ");
  _map_ok = true;
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
  _has_odom = true;

  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

int i = 0;
void pubSensedPoints() {
  pcl::toROSMsg(cloudMap_filter, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);

  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "read_pcd");
  ros::NodeHandle n("~");

  _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  // _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
_all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/globalmap", 1);

  _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

  click_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);
  // ros::Subscriber click_sub = n.subscribe("/goal", 10, clickCallback);

  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/circle_num", circle_num_, 30);

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);

  n.param("ObstacleShape/radius_l", radius_l_, 7.0);
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);
  n.param("ObstacleShape/z_l", z_l_, 7.0);
  n.param("ObstacleShape/z_h", z_h_, 7.0);
  n.param("ObstacleShape/theta", theta_, 7.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/radius", _sense_rate, 10.0);
  n.param("map/map_file", mapFile, std::string("/home/xulong/diablo-planner/src/read_pcd/down.pcd"));

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  ros::Rate loop_rate(1);

  while (ros::ok())
  // while (t--)
   {
    if (_map_ok)
    {
      pubSensedPoints();
    }else
    {
      RandomMapGenerate();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
