#include <ros/ros.h>
#include <Eigen/Eigen> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

#include <stdlib.h>
#include <time.h>

ros::Publisher global_map_pub;
ros::Publisher pose_pub;
double cloud_resolution;
pcl::PointCloud<pcl::PointXYZ> global_map_pcl_cloud;

using namespace Eigen;


void geneWall(double ori_x , double ori_y , double length, double width,double height)
{
    pcl::PointXYZ  s_point;

    for( double t_z = 0.0; t_z < height ; t_z += cloud_resolution )
    {
        for( double t_y = ori_y; t_y < ori_y + width ; t_y += cloud_resolution )
        {
            for( double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution)
            {
                s_point.x = t_x + (rand() % 10) / 250.0 ;
                s_point.y = t_y + (rand() % 10) / 250.0 ;
                s_point.z = t_z + (rand() % 10) / 800.0 ;

                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneWall(double ori_x , double ori_y ,double ori_z, double length, double width,double height)
{
    pcl::PointXYZ  s_point;

    for( double t_z = ori_z  ; t_z < height +ori_z  ; t_z += cloud_resolution )
    {
        for( double t_y = ori_y; t_y < ori_y + width ; t_y += cloud_resolution )
        {
            for( double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution)
            {
                s_point.x = t_x + (rand() % 10) / 250.0 ;
                s_point.y = t_y + (rand() % 10) / 250.0 ;
                s_point.z = t_z + (rand() % 10) / 800.0 ;

                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneTrangle(double ori_x , double ori_y, double height, double depth, double length)
{
    pcl::PointXYZ s_point;
    for(double t_x = ori_x; t_x < ori_x + depth; t_x += cloud_resolution)
    {   
        for(double t_y = ori_y ;  t_y < ori_y + length; t_y += cloud_resolution)
        {
            for(double t_z = 0.0 ;  t_z < (length - t_y + ori_y)*height/length ; t_z += cloud_resolution / 3.0)
            {
                if( abs(t_x - ori_x) >= 0.2 && abs(t_x - ori_x - depth) >= 0.2  &&
                    abs(t_y - ori_y) >= 0.2 && abs(t_y - ori_y - length) >= 0.2 &&
                    abs(t_z - 0.0) >= 0.2 && abs(t_z - (length - t_y + ori_y)*height/length) >= 0.2){continue;}
                s_point.x = t_x + (rand() % 10) / 250.0 ;
                s_point.y = t_y + (rand() % 10) / 250.0 ;
                s_point.z = t_z + (rand() % 10) / 800.0 ;
                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneTrangle(double ori_x , double ori_y, double ori_z, double height, double depth, double length)
{
    pcl::PointXYZ s_point;
    for(double t_x = ori_x; t_x < ori_x + depth; t_x += cloud_resolution)
    {   
        for(double t_y = ori_y ;  t_y < ori_y + length; t_y += cloud_resolution)
        {
            for(double t_z = ori_z ;  t_z < ori_z + (length - t_y + ori_y)*height/length ; t_z += cloud_resolution / 3.0)
            {
                if( abs(t_x - ori_x) >= 0.2 && abs(t_x - ori_x - depth) >= 0.2  &&
                    abs(t_y - ori_y) >= 0.2 && abs(t_y - ori_y - length) >= 0.2 &&
                    abs(t_z - 0.0) >= 0.2 && abs(t_z - ori_z - (length - t_y + ori_y)*height/length) >= 0.2){continue;}
                s_point.x = t_x + (rand() % 10) / 250.0 ;
                s_point.y = t_y + (rand() % 10) / 250.0 ;
                s_point.z = t_z + (rand() % 10) / 800.0 ;
                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneSinPlane(double ori_x, double ori_y, double c_z , double end_x, double end_y, double t, double h)
{
    pcl::PointXYZ s_point;
    double z,dz;
    for(double t_x = ori_x; t_x < end_x; t_x += cloud_resolution)
    {   
        for(double t_y = ori_y ;  t_y < end_y; t_y += cloud_resolution)
        {   
            dz =  h * sin(t*t_x ) + h* sin(t*t_y);
            z  =  c_z + dz;
            if(z < c_z) { z = c_z;}

            s_point.x = t_x + (rand() % 10) / 250.0 ;
            s_point.y = t_y + (rand() % 10) / 250.0 ;
            s_point.z = z + (rand() % 10) / 800.0 ;
            global_map_pcl_cloud.push_back(s_point);
        }
    }
}

void geneRoad(Vector3d start_pt, Vector3d end_pt, double width){

    pcl::PointXYZ s_point;
    Vector3d dir = end_pt - start_pt;
    double length = dir.norm();
    double t_step = cloud_resolution/length;
    double k_step = cloud_resolution/width;
    Vector3d expand(-dir(1), dir(0), 0);
    expand.normalize();
    expand *= width;

    Vector3d paver_pos, paver_hand_pos;    
    for(double t = 0; t <= 1 ; t += t_step)
    {
        paver_pos = start_pt + t * dir;
        for(double k = -0.5; k <= 0.5 ; k += k_step)
        {
            paver_hand_pos = paver_pos + k*expand;
            s_point.x = paver_hand_pos(0) + (rand() % 10) / 250.0 ;
            s_point.y = paver_hand_pos(1) + (rand() % 10) / 250.0 ;
            s_point.z = paver_hand_pos(2) + (rand() % 10) / 800.0 ;
            global_map_pcl_cloud.push_back(s_point);
        }
    }



}

void geneSpiral3D(double center_x, double center_y, double ori_z, double end_z, double radius, double width, double t)
{
    pcl::PointXYZ s_point;
    double phi = 0;
    for(double t_z = ori_z; t_z < end_z; t_z += cloud_resolution / (6*t))
    {   
        phi = t * (t_z - ori_z);
        for(double w = radius ;  w < radius + width; w += cloud_resolution)
        {    
            s_point.x = center_x + w*sin(phi) + (rand() % 10) / 250.0 ;
            s_point.y = center_y + w*cos(phi) + (rand() % 10) / 250.0 ;
            s_point.z = t_z + (rand() % 10) / 800.0 ;
            global_map_pcl_cloud.push_back(s_point);
        }
    }
}

void geneCylidar(double center_x, double center_y, double from_z , double end_z , double r)
{
    pcl::PointXYZ s_point;
    for(double t_z = from_z; t_z < end_z; t_z += cloud_resolution)
    {   
        for(double phi = 0 ;  phi < 6.2831; phi += cloud_resolution/r)
        {    
            s_point.x = center_x + r*cos(phi) + (rand() % 10) / 250.0 ;
            s_point.y = center_y + r*sin(phi) + (rand() % 10) / 250.0 ;
            s_point.z = t_z + (rand() % 10) / 800.0 ;
            global_map_pcl_cloud.push_back(s_point);
        }
    }
}

void map1Gene()
{
    geneWall(0,-0.3, 10, 13.5, 0.1);

    geneWall(0,3,0.95, 4, 3, 1.0);
    

    geneWall(0,0, 1.9 ,8, 2, 0.1);
    geneWall(0,2, 1.9 ,5, 11.5, 0.1);


    geneTrangle(5,2, 2.0,2.0,8.0);
    
    geneWall(0,0, 3.9 ,8, 2, 0.1);
    //geneWall(0,2, 4.0 ,5, 10, 0.1);
    geneTrangle(1,2,2.0, 2.0,2.0,8.0);

    geneTrangle(5,2,4.0, 2.0,2.0,8.0);
    geneWall(0,0, 5.9 ,8, 2, 0.1);
    geneWall(3,2, 3.9 ,2, 10, 0.1);
    geneWall(5,10, 3.9 ,2, 2, 0.1);

    geneWall(0,0, 8.0 ,0.1, 0.1, 0.1);

    geneWall(5.0 ,2.0,0.1,8.0,4.0);
    geneWall(7.0 ,2.0,0.1,8.0,2.0);

    geneWall(1.0 ,2.0,2.0, 0.1,8.0,2.0);
    geneWall(3.0 ,2.0,2.0, 0.1,8.0,2.0);
    


    //geneTrangle(3,4, 2.0,2.0,6.0);

    //geneTrangle(1,6, 2.0,2.0,4.0);
}

void map2Gene()
{
    geneWall(0,0, 20, 15, 0.1);

    //geneWall(0,0, 9.9 ,11, 6, 0.1);

    //geneSpiral3D(3,3,0,15, 1.5,2, 2);
    //geneSpiral3D(13,3,0,15, 1.5,2, 2);
    geneSpiral3D(7,8,0,12.4, 1.5,2, 2);
    geneCylidar(7,8,0,5,1.5);

    geneWall(0,0, 21.0 ,0.1, 0.1, 0.1);


    geneWall(6.11, 9.4, 12.3 ,10.0, 2.0, 0.1);

}

void map3Gene()
{
    geneSinPlane(0,0, -1.2, 15, 15, 1.5,0.6);
    geneWall(0,0, 6.0 ,0.1, 0.1, 0.1);
}

void map4Gene()
{
    geneWall(0,0, 30, 35, 0.1);
    geneWall(0,0, 21.0 ,0.1, 0.1, 0.1);

    for(int i = 1 ; i < 29 ; i += 8)
    {
        geneRoad(Vector3d(i,0,3), Vector3d(i,35,3), 2.0);

        if(i <= 10)
        {
            geneRoad(Vector3d(i+1,5,3), Vector3d(i+12,5,6), 2.0);
            geneRoad(Vector3d(i+1,15,3), Vector3d(i+12,15,6), 2.0);
            geneRoad(Vector3d(i+1,25,3), Vector3d(i+12,25,6), 2.0);
        }
    }
    for(int i = 6 ; i < 29 ; i += 8)
    {
        geneRoad(Vector3d(i,0,6), Vector3d(i,35,6), 2.0);
    }
    for(int i = 1 ; i < 35 ; i += 16)
    {
        geneRoad(Vector3d(0,i,3), Vector3d(30,i,3), 2.0);

        if(i < 32){
            geneRoad(Vector3d(5,i+13,0), Vector3d(5,i+1,3), 2.0);
            geneRoad(Vector3d(15,i+13,0), Vector3d(15,i+1,3), 2.0);
            geneRoad(Vector3d(10,i+13,0), Vector3d(10,i+1,3), 2.0);
            geneRoad(Vector3d(20,i+13,0), Vector3d(20,i+1,3), 2.0);
            geneRoad(Vector3d(0,i,6), Vector3d(30,i,6), 2.0);
        }
    }
}

void map5Gene()
{
    geneWall(0,0, 17, 10, 0.1);
    geneWall(0,0, 21.0 ,0.1, 0.1, 0.1);

    for(int i = 1 ; i < 5; i += 5)
    {
        geneRoad(Vector3d(i+16,0,3), Vector3d(i+16,10,3), 2.0);
        if(i < 10){
            geneRoad(Vector3d(i+1,2,0), Vector3d(i+15,2,3), 2.0);
            geneRoad(Vector3d(i+1,8,0), Vector3d(i+15,8,3), 2.0);
        }
    }

}

void pubGlobalMap()
{
    sensor_msgs::PointCloud2          global_map_cloud;

    map1Gene();    

    pcl::toROSMsg(global_map_pcl_cloud, global_map_cloud);
    global_map_cloud.header.frame_id = "world";
    global_map_cloud.header.frame_id = "camera_rgb_optical_frame";
    global_map_pub.publish(global_map_cloud);
    
    geometry_msgs::TransformStamped pose;
    pose.transform.translation.x = 0;
    pose.transform.translation.y = 0;
    pose.transform.translation.z = 0;
    pose.transform.rotation.x = 0;
    pose.transform.rotation.y = 0;
    pose.transform.rotation.z = 0;
    pose.transform.rotation.w = 1;
    pose_pub.publish(pose);

    ROS_INFO("global map published! ");
}


int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "globalmap_generator"); 
    ros::NodeHandle nh; 

    nh.param("cloud_resolution", cloud_resolution, 0.07);

    global_map_pub      = nh.advertise<sensor_msgs::PointCloud2>("globalmap", 5); 
    pose_pub            = nh.advertise<geometry_msgs::TransformStamped>("/kinect/vrpn_client/estimated_transform", 5); 

    int t = 3;
    while(t--)
    {
        pubGlobalMap();
        ros::Duration(1).sleep();
    }
    ros::spin();
    return 0;
}