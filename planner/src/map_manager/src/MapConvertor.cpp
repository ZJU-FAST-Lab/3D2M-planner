#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>

#define SAVE_MAP


ros::Publisher  pcmap_pub;

void mapSub(const std_msgs::Float32MultiArray& unity_map)
{

    sensor_msgs::PointCloud2 pcmap;
    pcl::PointCloud<pcl::PointXYZI> pclmap;
    pcl::PointXYZI pt;

    int N = unity_map.data.size() - 3;
    std::cout<<"N = "<<N<<std::endl;

    for (int i = 0 ; i < floor( N / 3) ; i++)
	{
		pt.x = unity_map.data[i*3];
		pt.y = unity_map.data[i*3+1];
		pt.z = unity_map.data[i*3+2];
        pclmap.points.push_back(pt);

	}

    std::cout<<"Convert map"<<std::endl;

    pcl::toROSMsg( pclmap, pcmap);
    pcmap.header.frame_id = "world";
    pcmap_pub.publish(pcmap);

#ifdef SAVE_MAP
    std::string filename("/home/lantern/ROS_workspace/diablo-planner/src/test.pcd");
    pcl::PCDWriter writer;
    writer.write(filename, pclmap, true);
#endif


}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "map_convertor_node");
  ros::NodeHandle nh("~");

  pcmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 100000);
  ros::Subscriber unity_map_sub = nh.subscribe("/unitymap", 100000 , mapSub);

  ros::spin();

  return 0;
}