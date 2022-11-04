#include "map_manager/PCSmap_manager.h"

void SurFelMap::init(ros::NodeHandle& nh)
{
    surfelmap_vis_pub = nh.advertise<visualization_msgs::Marker>("surfel_map_vis",10000);
}


void SurFelMap::appendSurfel(Vector3d pos, Vector3d ang)
{
    ang.normalize();
    SurFel::Ptr surfel(new SurFel(pos, ang) );
    surfels.push_back( surfel );
}

void SurFelMap::renderSurfelMap()
{
    int id = 1;
    visualization_msgs::Marker arrow, circle;
    circle.header.frame_id = arrow.header.frame_id  = "world";
    arrow.type             = visualization_msgs::Marker::ARROW;
    circle.type            = visualization_msgs::Marker::LINE_STRIP;
    circle.action          = arrow.action           = visualization_msgs::Marker::ADD;

    circle.color.r             = 1.0;
    circle.color.g             = 1.0;
    circle.color.b             = 1.0;
    circle.color.a             = 1.0;

    arrow.color.r              = 1.0;
    arrow.color.g              = 0.9;
    arrow.color.b              = 0.9;
    arrow.color.a              = 1.0;

    arrow.scale.x              = 0.2;
    arrow.scale.y              = 0.02;
    arrow.scale.z              = 0.02;

    circle.scale.x             = 0.03;
    circle.scale.y             = 0.03;
    circle.scale.z             = 0.03;


    Vector3d point_pos;
    geometry_msgs::Point pt;
    double r = 0.05;
    for( int i = 0 ; i < surfels.size(); i++ ){
        // std::cout<<"surfel = "<<surfels[i] -> position<<std::endl;
        circle.header.stamp    = arrow.header.stamp     = ros::Time::now();
        arrow.id               = id;
        circle.id              = id + 100000;
        id ++;

        arrow.pose.position.x      = (double)surfels[i] -> position(0);
        arrow.pose.position.y      = (double)surfels[i] -> position(1);
        arrow.pose.position.z      = (double)surfels[i] -> position(2);

        AngleAxisd t_V( M_PI , 0.5 * ( Vector3d(1,0,0) + surfels[i] -> pose ) );
        Quaterniond q( t_V );
        arrow.pose.orientation.w = q.w();
        arrow.pose.orientation.x = q.x();
        arrow.pose.orientation.y = q.y();
        arrow.pose.orientation.z = q.z();

        // circle.points.clear();
        // for(double t = 0.0 ; t <= 2*pi ; t += 0.2*pi)
        // {
        //     point_pos = surfels[i] -> position + r*cos(t)*surfels[i] -> a + r*sin(t)*surfels[i] -> b;
        //     pt.x = point_pos(0);
        //     pt.y = point_pos(1);
        //     pt.z = point_pos(2);
        //     circle.points.push_back(pt);
        // }

        surfelmap_vis_pub.publish(arrow);
        // surfelmap_vis_pub.publish(circle);

    }
    


}