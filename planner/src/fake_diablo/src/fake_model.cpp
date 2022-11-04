#include <fake_diablo/fake_model.hpp>

void ModelManager::init(ros::NodeHandle &nh)
{
    /*  param  */
    nh.param("ugv/ugv_l",ugv_l,0.6);
    nh.param("ugv/ugv_w",ugv_w,0.4);
    nh.param("ugv/ugv_h",ugv_h,0.3);
    nh.param("ugv/mesh" ,mesh_resource,   std::string("package://fake_diablo/param/car.dae"));
    nh.param("ugv/mesh2" ,mesh_resource2, std::string("package://fake_diablo/param/car.dae"));
    nh.param("ugv/mesh3" ,mesh_resource3, std::string("package://fake_diablo/param/car.dae"));
    nh.param("ugv/mesh4" ,mesh_resource4, std::string("package://fake_diablo/param/car.dae"));
    nh.param("ugv/frame",frame,std::string("world"));

    nh.param("max_height",max_height, 1.0);

    /* callback */
    visugv_pub    = nh.advertise<visualization_msgs::Marker>("odom_mesh", 100,true);
    waypoints_sub = nh.subscribe("waypoints", 1, &ModelManager::rcvWaypointsCallback, this);
    odom_sub      = nh.subscribe("odom", 1, &ModelManager::odomCallback, this);
}

void ModelManager::odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
    //static double t = 0;
    //t += 0.01;
    //if (t > 0.4){t = 0;}
    double floor_h = 0;
    static double w_ang = 0;
    w_ang +=  odom->twist.twist.angular.y * 0.1 ;
    floor_h = odom->twist.twist.angular.z;
    //w_ang += 0.05;
    visualization_msgs::Marker WpMarker;
    WpMarker.id               = 0;
    WpMarker.header.stamp     = ros::Time::now();
    WpMarker.header.frame_id  = "world";
    WpMarker.action           = visualization_msgs::Marker::ADD;
    WpMarker.type             = visualization_msgs::Marker::MESH_RESOURCE;
    WpMarker.ns               = "ugv_mesh";
    WpMarker.mesh_use_embedded_materials = true;
    WpMarker.color.r          = 0.0;
    WpMarker.color.g          = 0.0;
    WpMarker.color.b          = 0.0;
    WpMarker.color.a          = 0.0;
    WpMarker.scale.x          = ugv_l/4.5;
    WpMarker.scale.y          = ugv_l/4.5;
    WpMarker.scale.z          = ugv_l/4.5;

    Eigen::Quaterniond q( odom->pose.pose.orientation.w, 
                          odom->pose.pose.orientation.x, 
                          odom->pose.pose.orientation.y,
                          odom->pose.pose.orientation.z );

    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    Eigen::Matrix3d       R(q);
		double odom_yaw 	    = atan2(R.col(0)[1],R.col(0)[0]); 
    if(odom_yaw > 0){eulerAngle[0] *= -1;} 

    Eigen::AngleAxisd rollTrans(Eigen::AngleAxisd( eulerAngle[1],Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchTrans(Eigen::AngleAxisd(-eulerAngle[0],Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawTrans(Eigen::AngleAxisd(eulerAngle[2],Eigen::Vector3d::UnitZ())); 
    Eigen::Quaterniond qtr = yawTrans * pitchTrans * rollTrans;

    //std::cout<<"yaw = " << eulerAngle[0] <<", odom_yaw = " << odom_yaw<<std::endl;

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd( M_PI/2,Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())); 
    Eigen::Quaterniond qder = yawAngle * pitchAngle * rollAngle;

    //Eigen::Quaterniond qz(cos(0),0,0,sin(0));
    //Eigen::Quaterniond qx(sin(t),0,0,cos(t));
    //std::cout<<"theta = " <<t<<std::endl;
    //Eigen::Quaterniond qy(0, sin(t),0,cos(t));

    double d = max_height - odom -> pose.pose.position.z;
    //d = t;
    //Eigen::Quaterniond qx0(cos(-d/1.5),sin(-d/1.5),0,0);
    //Eigen::Quaterniond qx1(cos(d/1.5),sin(d/1.5),0,0);

    Eigen::AngleAxisd rollDir(Eigen::AngleAxisd( 0,Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchDir0(Eigen::AngleAxisd(d,Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond qdir0 = pitchDir0 * rollDir;
    Eigen::AngleAxisd pitchDir1(Eigen::AngleAxisd(-2*d,Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond qdir1 = pitchDir1 * rollDir;
    Eigen::AngleAxisd pitchDir2(Eigen::AngleAxisd(2*d + w_ang ,Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond qdir2 = pitchDir2 * rollDir;

    double jump_height = odom->twist.twist.angular.x;

    double z_buf = ( 0.5 * jump_height ) > (max_height - 0.8) ? (max_height - 0.8) : ( 0.5 * jump_height ) ;
    d += 0.5*z_buf;

    Eigen::Quaterniond q0,q1,q2;
    q0 = qder * qtr * qdir1;
    WpMarker.pose.orientation.w = q0.w();
    WpMarker.pose.orientation.x = q0.x();
    WpMarker.pose.orientation.y = q0.y();
    WpMarker.pose.orientation.z = q0.z();  //leg
    WpMarker.pose.position.x      = odom -> pose.pose.position.x + 1.0 * sin(odom_yaw) + 0.5*d * cos(odom_yaw);
    WpMarker.pose.position.y      = odom -> pose.pose.position.y - 1.0 * cos(odom_yaw) + 0.5*d * sin(odom_yaw);
    WpMarker.pose.position.z      = jump_height + 0.7*d   - z_buf      + floor_h;
    WpMarker.mesh_resource      = mesh_resource2;
    visugv_pub.publish(WpMarker);

    //WpMarker.pose.position.x += d;
    q1 = qder * qtr ;//* qdir1;
    WpMarker.pose.orientation.w = q1.w();
    WpMarker.pose.orientation.x = q1.x();
    WpMarker.pose.orientation.y = q1.y();
    WpMarker.pose.orientation.z = q1.z();
    WpMarker.mesh_resource      = mesh_resource;   //height_pure
    WpMarker.pose.position.x      = odom -> pose.pose.position.x + 0.2 * sin(odom_yaw);
    WpMarker.pose.position.y      = odom -> pose.pose.position.y - 0.2 * cos(odom_yaw);
    WpMarker.pose.position.z      = jump_height - d + 0.5   -z_buf      + floor_h;
    WpMarker.id               = 1;
    visugv_pub.publish(WpMarker);

    q2 = qder * qtr * qdir0;
    WpMarker.pose.orientation.w = q2.w();
    WpMarker.pose.orientation.x = q2.x();
    WpMarker.pose.orientation.y = q2.y();
    WpMarker.pose.orientation.z = q2.z();
    WpMarker.mesh_resource      = mesh_resource3; // mid
    WpMarker.pose.position.x      = odom -> pose.pose.position.x + 1.0 * sin(odom_yaw) - 0.8*d * cos(odom_yaw);
    WpMarker.pose.position.y      = odom -> pose.pose.position.y - 1.0 * cos(odom_yaw) - 0.8*d * sin(odom_yaw);
    WpMarker.pose.position.z      = jump_height - 1.4 * d   - z_buf     + floor_h;
    WpMarker.id               = 2;
    visugv_pub.publish(WpMarker);

    q2 = qder * qtr * qdir2;
    WpMarker.pose.orientation.w = q2.w();
    WpMarker.pose.orientation.x = q2.x();
    WpMarker.pose.orientation.y = q2.y();
    WpMarker.pose.orientation.z = q2.z();
    WpMarker.mesh_resource      = mesh_resource4; // wheels

    //static double r = 0.4;
    //static double a = 3;
    //if(w_ang > M_PI * 2){
    //   a -= 0.001;   w_ang -= 2 * M_PI; 
    //   if(a < 2.0){a = 3; r += 0.01;}
    //   std::cout<<a<< "\t" <<r<<std::endl;
    //}

    double a = 2.7, r = 0.4;

    WpMarker.pose.position.x      = odom -> pose.pose.position.x + 1.0 * sin(odom_yaw) + 0.2 * d * cos(odom_yaw) + r * (cos(a) -  cos(a - w_ang - 2*d)) * cos(odom_yaw);
    WpMarker.pose.position.y      = odom -> pose.pose.position.y - 1.0 * cos(odom_yaw) + 0.2 * d * sin(odom_yaw) + r * (cos(a) -  cos(a - w_ang - 2*d)) * sin(odom_yaw);
    WpMarker.pose.position.z      = jump_height    - z_buf + r * (sin(a) -  sin(a - w_ang - 2*d))      + floor_h;
    WpMarker.id               = 3;
    visugv_pub.publish(WpMarker);

    //for(int i = 0 ; i < 8; i++)
    //{ 
    //  double t_ang = M_PI * i/4.0;
    //  Eigen::AngleAxisd pitchDirt(Eigen::AngleAxisd(t_ang ,Eigen::Vector3d::UnitZ()));
    //  Eigen::Quaterniond qdirt = pitchDirt * rollDir;
    //  q2 = qder * qtr * qdirt;
    //  WpMarker.pose.orientation.w = q2.w();
    //  WpMarker.pose.orientation.x = q2.x();
    //  WpMarker.pose.orientation.y = q2.y();
    //  WpMarker.pose.orientation.z = q2.z();
    //  WpMarker.pose.position.x      = odom -> pose.pose.position.x + 1.0 * sin(odom_yaw) - 0.2 * d * cos(odom_yaw) + r * (cos(a) -  cos(a - t_ang));
    //  WpMarker.pose.position.y      = odom -> pose.pose.position.y - 1.0 * cos(odom_yaw) - 0.2 * d * sin(odom_yaw) ;
    //  WpMarker.pose.position.z      = jump_height - 0.7 * d   - z_buf + r * (sin(a) -  sin(a - t_ang));
    //  WpMarker.id               = 4 + i;
    //  visugv_pub.publish(WpMarker);
    //}
    
}



void ModelManager::rcvWaypointsCallback(const geometry_msgs::PoseStamped& msg)
{
   std::cout<<"manager get waypoints!"<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_model_vis");
  ros::NodeHandle nh("~");
  ModelManager diablo;
  diablo.init(nh);
  ros::spin();
  return 0;
}