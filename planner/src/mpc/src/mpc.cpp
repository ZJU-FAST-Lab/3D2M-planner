#include "mpc.h"
 
using namespace std;

pcl::PointCloud<pcl::PointXY>::Ptr cloudMapZ(new pcl::PointCloud<pcl::PointXY>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXY> kdtree;

void MPC::init(ros::NodeHandle &nh)
{
    nh.param("mpc/du_threshold", du_th, -1.0);
    nh.param("mpc/dt", dt, -1.0);
    nh.param("mpc/max_iter", max_iter, -1);
    nh.param("mpc/predict_steps", T, -1);
    nh.param("mpc/delay_num", delay_num, -1);
    nh.param("mpc/max_omega", max_omega, -1.0);
    nh.param("mpc/max_domega", max_domega, -1.0);
    nh.param("mpc/max_speed", max_speed, -1.0);
    nh.param("mpc/max_accel", max_accel, -1.0);
    nh.param("mpc/tolerance", tolerance, 0.1);
    nh.param("mpc/tv_dist", tv_dist, -1.0);
    nh.param("mpc/in_test", in_test, false);
    nh.param("mpc/in_mocap", in_mocap, false);
    nh.param("mpc/in_bk", in_bk, false);
    nh.param("mpc/control_a", control_a, false);
    nh.param<std::vector<double>>("mpc/matrix_q", Q, std::vector<double>());
    nh.param<std::vector<double>>("mpc/matrix_r", R, std::vector<double>());
    nh.param<std::vector<double>>("mpc/matrix_rd", Rd, std::vector<double>());
    nh.param<string>("mpc/test_traj", test_traj, "xxx");

    cout<<"du_t="<<du_th<<endl;

    has_odom = false;
    receive_traj_ = false;
    max_comega = max_domega * dt;
    max_cv = max_accel * dt;
    xref = Eigen::Matrix<double, 4, 500>::Zero(4, 500);
    last_output = output = dref = Eigen::Matrix<double, 2, 500>::Zero(2, 500);
    for (int i=0; i<delay_num; i++)
        output_buff.push_back(Eigen::Vector2d::Zero());
    cmd.height = 1.0;
    cmd.height_vel = 0.0;
    cmd.omega = 0.0;
    cmd.pitch = 0.0;
    cmd.pitch_vel = 0.0;
    cmd.roll = 0.0;
    cmd.speed = 0.0;

    has_cloud = false;

    pos_cmd_pub_     = nh.advertise<diablo_sdk::Diablo_Ctrl>("cmd", 200);
    pos_cmd_pub_geo_ = nh.advertise<geometry_msgs::Pose>("cmd_geo", 200);
    vis_pub = nh.advertise<visualization_msgs::Marker>("/following_path", 10);
    predict_pub = nh.advertise<visualization_msgs::Marker>("/predict_path", 10);
    ref_pub = nh.advertise<visualization_msgs::Marker>("/reference_path", 10);
    cmd_timer_ = nh.createTimer(ros::Duration(0.01), &MPC::cmdCallback, this);
    odom_sub_ = nh.subscribe("odom", 1, &MPC::rcvOdomCallBack, this);
    traj_sub_ = nh.subscribe("traj", 1, &MPC::rcvTrajCallBack, this);
    tv_sub_ = nh.subscribe("/TV_path", 1, &MPC::rcvTVPathCallBack, this);
    cloud_sub_ = nh.subscribe("/cloud_pcd", 1, &MPC::rcvCloudCallBack, this);
    cmd_geo_pub   = nh.advertise<geometry_msgs::Pose>("cmd_geo", 200);
    err_pub = nh.advertise<std_msgs::Float64>("/track_err", 10);

    if (!in_mocap)
        vel_sub_ = nh.subscribe("vel", 1, &MPC::rcvVelCallBack, this);

    if (in_test || in_bk)
    {
        trigger_sub_ = nh.subscribe("/move_base_simple/goal", 1, &MPC::rcvTriggerCallBack, this);
        if (in_bk)
        {
            receive_traj_ = false;
            init_pub = nh.advertise<geometry_msgs::Pose>("/diablo_flash", 10);
            string o = test_traj;
            string find_data("data");
            size_t f_index = o.find(find_data) + 5;
            if (test_traj[f_index]=='l')
                in_test = true;
            else
                in_test = false;
            outfile.open(o.insert(f_index,"err_"), std::ofstream::out);
            outfile.clear();
            traj_analyzer.setTraj(test_traj.c_str());
        }
    }
}

void MPC::rcvCloudCallBack(sensor_msgs::PointCloud2ConstPtr msg)
{
    if (has_cloud)
        return;
    pcl::fromROSMsg(*msg, *cloudMap);
    for (size_t i=0; i<cloudMap->points.size();i++)
    {
        pcl::PointXY p;
        p.x = cloudMap->points[i].x;
        p.y = cloudMap->points[i].y;
        cloudMapZ->points.push_back(p);
    }
    kdtree.setInputCloud(cloudMapZ);
    has_cloud = true;
    ROS_WARN("RcvCloud!");
}

void MPC::rcvTVPathCallBack(nav_msgs::PathConstPtr msg)
{
    std::vector<double> x_;
    std::vector<double> y_;
    for (size_t i=msg->poses.size()-1; i>0; i--)
    {
        if (!x_.empty())
        {
            Eigen::Vector2d now_p(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y);
            Eigen::Vector2d last_p(x_.back(), y_.back());
            if ((now_p-last_p).norm()>tv_dist)
            {
                x_.push_back(msg->poses[i].pose.position.x);
                y_.push_back(msg->poses[i].pose.position.y);
            }
        }
        else
        {
            x_.push_back(msg->poses[i].pose.position.x);
            y_.push_back(msg->poses[i].pose.position.y);
        }        
    }
    in_test = true;
    csp = cubic_spline_planner(x_, y_, 0.1);
    csp_path.clear();
    csp_path = csp.get_path();
    csp_path_3d.clear();
    for (size_t i=0; i<csp_path.size(); i++)
    {
        Eigen::Vector3d node = csp_path[i];
        pcl::PointXY p;
        p.x = node[0];
        p.y = node[1];
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        kdtree.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        
        node[2] = cloudMap->points[pointIdxNKNSearch[0]].z;
        csp_path_3d.push_back(node);
    }
    for (size_t i=0; i<csp_path_3d.size(); i++)
    {
        double zs = 0;
        if (i+40 > csp_path_3d.size())
        {
            size_t j=0;
            for (; i+j < csp_path_3d.size(); j++)
            {
                zs += csp_path_3d[i+j][2];
            }
            zs /= (double)j;
        }else
        {
            size_t j=0;
            for (; j<40; j++)
            {
                zs += csp_path_3d[i+j][2];
            }
            zs /= (double)j;
        }
        if (zs > 15)
            csp_path_3d[i][2] = csp_path_3d[0][2];
        else
            csp_path_3d[i][2] = zs;
    }
    traj_duration_ = csp.get_duration();
    start_time_ = ros::Time::now();
    t_track = 0.0;
    receive_traj_ = true;
}

void MPC::rcvTriggerCallBack(const geometry_msgs::PoseStamped msg)
{
    if (in_bk)
    {
        Eigen::Vector3d initp = traj_analyzer.nextInitPointMinco();
        geometry_msgs::Pose msg;

        pcl::PointXY p;
        p.x = initp[0];
        p.y = initp[1];
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        kdtree.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        
        msg.position.x = initp(0);
        msg.position.y = initp(1);
        msg.position.z = cloudMap->points[pointIdxNKNSearch[0]].z + 0.8;
        init_pub.publish(msg);
        traj_analyzer.beginNextTrajMinco();
        receive_traj_ = true;
    }
    else
    {
        // csp_path = csp.get_path();
        // traj_duration_ = csp.get_duration();
        // start_time_ = ros::Time::now();
        // t_track = 0.0;
        // receive_traj_ = true;
    }
    
}

void MPC::rcvTrajCallBack(mpc::PolynomeConstPtr msg)
{
    in_test = false;
    receive_traj_ = true;
    traj_analyzer.setTraj(msg);
}

void MPC::rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
    has_odom = true;
    now_state.x = msg->pose.pose.position.x;
    now_state.y = msg->pose.pose.position.y;
    now_state.z = msg->pose.pose.position.z;

    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z);
    Eigen::Matrix3d R(q);
    now_state.theta = atan2(R.col(0)[1],R.col(0)[0]);
    if (in_mocap)
    {
        Eigen::Vector2d lvel(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
        now_state.v = lvel.norm();
    }
    
    // double direction = atan2(lvel(1), lvel(0));
    // if ((direction-now_state.theta)>M_PI/2)
    // {
    //     now_state.v = -now_state.v;
    // }
}

void MPC::rcvVelCallBack(diablo_sdk::OSDK_LEGMOTORSConstPtr msg)
{
    now_state.v = (msg->left_wheel_vel + msg->right_wheel_vel) * 0.0935 / 2.0;
}

void MPC::cmdCallback(const ros::TimerEvent &e)
{
    // drawFollowPath();
    
    if (!has_odom || !receive_traj_)
        return;

    if (in_test && !in_bk)
    {
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - start_time_).toSec();

        // for test
        t_cur = get_nearest();
        // ROS_INFO("state: x=%f, y=%f, v=%f, theta=%f", now_state.x, now_state.y, now_state.v, now_state.theta);

        if (t_cur == traj_duration_ && (csp.get_state(t_cur).head(2) - Eigen::Vector2d(now_state.x, now_state.y)).norm() < tolerance)
        {
            t_cur+=1.0;
        }

        if (t_cur <= traj_duration_ && t_cur >= 0.0)
        {
            double t_temp = t_cur;
            for (int i=0; i<T; i++)
            {
                Eigen::Vector3d temp;

                if (now_state.v > 0)
                {
                    t_temp+=max_speed*dt;
                    // t_temp+=now_state.v*dt;
                }

                // double downvt = traj_duration_>3.0?traj_duration_ - 3.0:traj_duration_/2.0; 
                // double upv = traj_duration_>3.0?3.0:traj_duration_/2.0;
                double downvt = 0.0;
                double upv = 0.0;
                
                if (t_temp < upv && t_temp > 0.0)
                {
                    temp = csp.get_state(t_temp);
                    xref(2, i) = max(max_speed * t_temp / upv, max_speed*0.1);
                }
                else if (t_temp < downvt)
                {
                    temp = csp.get_state(t_temp);
                    xref(2, i) = max_speed;
                }
                else if (t_temp < traj_duration_)
                {
                    temp = csp.get_state(t_temp);
                    xref(2, i) = max_speed * (traj_duration_ - t_temp) / (traj_duration_ - downvt);
                }
                else
                {
                    temp = csp.get_state(traj_duration_);
                    xref(2, i) = 0.0;
                }

                xref(0, i) = temp[0];
                xref(1, i) = temp[1];
                xref(3, i) = temp[2];

                if (control_a)
                    dref(0, i) = 0.0;
                else
                    dref(0, i) = xref(2, i);
                
                dref(1, i) = 0.0;
            }
            smooth_yaw();
            getCmd();
        }
        else if (t_cur > traj_duration_)
        {
            cmd.speed = 0.0;
            cmd.omega = 0.0;
        }
        else
        {
            cout << "[Traj server]: invalid time." << endl;
        }
    }
    else if (in_test && in_bk)
    {  
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - start_time_).toSec();

        t_cur = get_nearest();

        if ((t_cur == traj_duration_ && (csp.get_state(t_cur).head(2) - Eigen::Vector2d(now_state.x, now_state.y)).norm() < tolerance)\
            ||(time_now - start_time_).toSec()>traj_duration_/max_speed*3.0)
        {
            // t_cur+=1.0;
            traj_analyzer.at_goal = true;
        }

        std_msgs::Float64 err;
        if (t_cur <= traj_duration_ && t_cur >= 0.0)
        {
            Eigen::Vector3d perr = csp.get_state(t_cur);
            err.data = sqrt(pow(perr[0] - now_state.x, 2) + pow(perr[1] - now_state.y, 2));
            err_pub.publish(err);
        }

        if (traj_analyzer.at_goal)
        {
            geometry_msgs::Pose cmd_geo;
            cmd_geo.position.x = 0;
            cmd_geo.position.y = 0;
            cmd_geo_pub.publish(cmd_geo);

            cmd.speed = 0.0;
            cmd.omega = 0.0;
            pos_cmd_pub_.publish(cmd);

            receive_traj_ = false;

            Eigen::Vector2d initp = traj_analyzer.nextInitPointLiuming();
            if (initp == Eigen::Vector2d::Zero())
                outfile.close();
            outfile<< (ros::Time::now()-start_time_).toSec() <<std::endl;

            in_test = true;
            std::vector<std::pair<double, double>> now_traj = traj_analyzer.beginNextTrajLiuming();
            std::vector<double> x_;
            std::vector<double> y_;
            for (size_t i=0; i<now_traj.size(); i++)
            {
                x_.push_back(now_traj[i].first);
                y_.push_back(now_traj[i].second);
            }
            csp = cubic_spline_planner(x_, y_, 0.1);
            csp_path.clear();
            csp_path = csp.get_path();
                        
            pcl::PointXY p;
            p.x = initp[0];
            p.y = initp[1];
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);
            kdtree.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
            
            geometry_msgs::Pose msg;
            msg.position.x = initp(0);
            msg.position.y = initp(1);
            msg.position.z = cloudMap->points[pointIdxNKNSearch[0]].z + 0.8;
            init_pub.publish(msg);
            traj_duration_ = csp.get_duration();
            start_time_ = ros::Time::now();
            t_track = 0.0;
            ros::Duration(5.0).sleep();
            traj_analyzer.at_goal = false;
            receive_traj_ = true;
        }
        else
        {
            outfile<<err.data<<",";
            if (t_cur <= traj_duration_ && t_cur >= 0.0)
            {
                double t_temp = t_cur;
                for (int i=0; i<T; i++)
                {
                    Eigen::Vector3d temp;

                    if (now_state.v > 0)
                    {
                        t_temp+=max_speed*dt;
                    }

                    // double downvt = traj_duration_>3.0?traj_duration_ - 3.0:traj_duration_/2.0; 
                    // double upv = traj_duration_>3.0?3.0:traj_duration_/2.0;
                    double downvt = 0.0;
                    double upv = 0.0;
                    
                    if (t_temp < upv && t_temp > 0.0)
                    {
                        temp = csp.get_state(t_temp);
                        xref(2, i) = max(max_speed * t_temp / upv, max_speed*0.1);
                    }
                    else if (t_temp < downvt)
                    {
                        temp = csp.get_state(t_temp);
                        xref(2, i) = max_speed;
                    }
                    else if (t_temp < traj_duration_)
                    {
                        temp = csp.get_state(t_temp);
                        xref(2, i) = max_speed * (traj_duration_ - t_temp) / (traj_duration_ - downvt);
                    }
                    else
                    {
                        temp = csp.get_state(traj_duration_);
                        xref(2, i) = 0.0;
                    }

                    xref(0, i) = temp[0];
                    xref(1, i) = temp[1];
                    xref(3, i) = temp[2];

                    if (control_a)
                        dref(0, i) = 0.0;
                    else
                        dref(0, i) = xref(2, i);
                    
                    dref(1, i) = 0.0;
                }
                smooth_yaw();
                getCmd();
            }
            else if (t_cur > traj_duration_)
            {
                cmd.speed = 0.0;
                cmd.omega = 0.0;
            }
        }
    }
    else if (!in_test && in_bk)
    {
        vector<TrajPoint> P = traj_analyzer.getRefPoints(T, dt);
        TrajPoint perr = traj_analyzer.getErrState();
        std_msgs::Float64 err;
        err.data = sqrt(pow(perr.x - now_state.x, 2) + pow(perr.y - now_state.y, 2));
        err_pub.publish(err);

        if (traj_analyzer.at_goal)
        {
            geometry_msgs::Pose cmd_geo;
            cmd_geo.position.x = 0;
            cmd_geo.position.y = 0;
            cmd_geo_pub.publish(cmd_geo);

            cmd.speed = 0.0;
            cmd.omega = 0.0;
            pos_cmd_pub_.publish(cmd);

            Eigen::Vector3d initp = traj_analyzer.nextInitPointMinco();
            if (initp == Eigen::Vector3d::Zero())
                outfile.close();
            outfile<<std::endl;
            
            pcl::PointXY p;
            p.x = initp[0];
            p.y = initp[1];
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);
            kdtree.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
            
            geometry_msgs::Pose msg;
            msg.position.x = initp(0);
            msg.position.y = initp(1);
            msg.position.z = cloudMap->points[pointIdxNKNSearch[0]].z + 0.8;
            init_pub.publish(msg);
            receive_traj_ = false;
            traj_analyzer.beginNextTrajMinco();
            receive_traj_ = true;
        }
        else
        {
            outfile<<err.data<<",";
            for (int i=0; i<T; i++)
            {
                xref(0, i) = P[i].x;
                xref(1, i) = P[i].y;
                xref(3, i) = P[i].theta;
                dref(0, i) = 0.0;
                dref(1, i) = 0.0;
            }
            
            smooth_yaw();
            getCmd();
        }
    }
    else if (!in_bk && !in_test)
    {
        vector<TrajPoint> P = traj_analyzer.getRefPoints(T, dt);

        if (traj_analyzer.at_goal)
        {
            cmd.speed = 0.0;
            cmd.omega = 0.0;
        }
        else
        {
            if (control_a)
            {
            
                for (int i=0; i<T; i++)
                {
                    xref(0, i) = P[i].x;
                    xref(1, i) = P[i].y;
                    xref(2, i) = P[i].v;
                    xref(3, i) = P[i].theta;
                    dref(0, i) = 0.0;
                    dref(1, i) = 0.0;
                }
            }
            else
            {
                for (int i=0; i<T; i++)
                {
                    xref(0, i) = P[i].x;
                    xref(1, i) = P[i].y;
                    xref(3, i) = P[i].theta;
                    dref(0, i) = 0.0;
                    dref(1, i) = 0.0;
                }
            }
            smooth_yaw();
            getCmd();
        }
    }

    geometry_msgs::Pose cmd_geo;

    cmd_geo.position.x = cmd.speed;
    cmd_geo.position.y = cmd.omega;
    cmd_geo.position.z = traj_analyzer.minco_traj.getPos((ros::Time::now() - traj_analyzer.start_time).toSec())(2) - now_state.z;

    pos_cmd_pub_.publish(cmd);
    pos_cmd_pub_geo_.publish(cmd_geo);
    
    // ROS_INFO("in MPC, the cmd is: a=%f, steer=%f", cmd.speed, cmd.omega);
}

void MPC::getLinearModel(const MPCState& s)
{
    if (control_a)
    {
        A = Eigen::Matrix4d::Identity();
        A(0, 2) = dt * cos(s.theta);
        A(1, 2) = dt * sin(s.theta);
        A(0, 3) = -s.v * A(1, 2);
        A(1, 3) = s.v * A(0, 2);

        B = Eigen::Matrix<double, 4, 2>::Zero();
        B(2, 0) = dt;
        B(3, 1) = dt;

        C = Eigen::Vector4d::Zero();
        C(0) = -A(0, 3) * s.theta; 
        C(1) = -A(1, 3) * s.theta;
    }
    else
    {
        B = Eigen::Matrix<double, 3, 2>::Zero();
        B(0, 0) = cos(s.theta) * dt;
        B(1, 0) = sin(s.theta) * dt;
        B(2, 1) = dt;

        A = Eigen::Matrix3d::Identity();
        A(0, 2) = -B(1, 0) * s.v;
        A(1, 2) = B(0, 0) * s.v;

        C = Eigen::Vector3d::Zero();
        C(0) = -A(0, 2) * s.theta; 
        C(1) = -A(1, 2) * s.theta; 
    }
}

void MPC::stateTrans(MPCState& s, double a, double yaw_dot)
{
    if (control_a)
    {
        if (yaw_dot >= max_omega)
        {
            yaw_dot = max_omega;
        }else if (yaw_dot<= - max_omega)
        {
            yaw_dot = -max_omega;
        }

        s.x = s.x + s.v * cos(s.theta) * dt;
        s.y = s.y + s.v * sin(s.theta) * dt;
        s.theta = s.theta + yaw_dot * dt;
        s.v = s.v + a * dt;

        if (s.v >= max_speed)
        {
            s.v = max_speed;
        }else if (s.v<= -max_speed)
        {
            s.v = -max_speed;
        }
    }
    else
    {
        if (yaw_dot >= max_omega)
        {
            yaw_dot = max_omega;
        }else if (yaw_dot<= - max_omega)
        {
            yaw_dot = -max_omega;
        }
        if (s.v >= max_speed)
        {
            s.v = max_speed;
        }else if (s.v<= -max_speed)
        {
            s.v = -max_speed;
        }

        s.x = s.x + a * cos(s.theta) * dt;
        s.y = s.y + a * sin(s.theta) * dt;
        s.theta = s.theta + yaw_dot * dt;
        s.v = a;
    }
}

void MPC::predictMotion(void)
{
    xbar[0] = now_state;

    MPCState temp = now_state;
    for (int i=1; i<T+1; i++)
    {
        stateTrans(temp, output(0, i-1), output(1, i-1));
        xbar[i] = temp;
    }
}

void MPC::predictMotion(MPCState* b)
{
    b[0] = xbar[0];

    Eigen::MatrixXd Ax;
    Eigen::MatrixXd Bx;
    Eigen::MatrixXd Cx;
    Eigen::MatrixXd xnext;
    MPCState temp = xbar[0];
    if (control_a)
        for (int i=1; i<T+1; i++)
        {
            Ax = Eigen::Matrix4d::Identity();
            Ax(0, 2) = dt * cos(xbar[i-1].theta);
            Ax(1, 2) = dt * sin(xbar[i-1].theta);
            Ax(0, 3) = -xbar[i-1].v * Ax(1, 2);
            Ax(1, 3) = xbar[i-1].v * Ax(0, 2);
            Ax(3, 2) = 0.0;

            Bx = Eigen::Matrix<double, 4, 2>::Zero();
            Bx(2, 0) = dt;
            Bx(3, 1) = dt;

            Cx = Eigen::Vector4d::Zero();
            Cx(0) = -Ax(0, 3) * xbar[i-1].theta;
            Cx(1) = -Ax(1, 3) * xbar[i-1].theta;
            
            xnext = Ax*Eigen::Vector4d(temp.x, temp.y, temp.v, temp.theta) + Bx*Eigen::Vector2d(output(0, i-1), output(1, i-1)) + Cx;
            temp.x = xnext(0);
            temp.y = xnext(1);
            temp.v = xnext(2);
            temp.theta = xnext(3);
            b[i] = temp;
        }
    else
         
        for (int i=1; i<T+1; i++)
        {  
            Bx = Eigen::Matrix<double, 3, 2>::Zero();
            Bx(0, 0) = cos(xbar[i-1].theta) * dt;
            Bx(1, 0) = sin(xbar[i-1].theta) * dt;
            Bx(2, 1) = dt;

            Ax = Eigen::Matrix3d::Identity();
            Ax(0, 2) = -Bx(1, 0) * xbar[i-1].v;
            Ax(1, 2) = Bx(0, 0) * xbar[i-1].v;

            Cx = Eigen::Vector3d::Zero();
            Cx(0) = -Ax(0, 2) * xbar[i-1].theta; 
            Cx(1) = -Ax(1, 2) * xbar[i-1].theta;
            xnext = Ax*Eigen::Vector3d(temp.x, temp.y, temp.theta) + Bx*Eigen::Vector2d(output(0, i-1), output(1, i-1)) + Cx;
            temp.x = xnext(0);
            temp.y = xnext(1);
            temp.theta = xnext(2);
            b[i] = temp;
        }

}

void MPC::solveMPCA(void)
{
    const int dimx = 4 * (T - delay_num);
    const int dimu = 2 * (T - delay_num);
    const int nx = dimx + dimu;

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // first-order
    for (int i=0, j=delay_num; i<dimx; i+=4, j++)
    {
        gradient[i] = -2 * Q[0] * xref(0, j);
        gradient[i+1] = -2 * Q[1] * xref(1, j);
        gradient[i+2] = -2 * Q[2] * xref(2, j);
        gradient[i+3] = -2 * Q[3] * xref(3, j);
    }

    // second-order
    const int nnzQ = nx + dimu - 2;
    int irowQ[nnzQ];
    int jcolQ[nnzQ];
    double dQ[nnzQ];
    for (int i=0; i<nx; i++)
    {
        irowQ[i] = jcolQ[i] = i;
    }
    for (int i=nx; i<nnzQ; i++)
    {
        irowQ[i] = i - dimu + 2;
        jcolQ[i] = i - dimu;
    }
    for (int i=0; i<dimx; i+=4)
    {
        dQ[i] = Q[0] * 2.0;
        dQ[i+1] = Q[1] * 2.0;
        dQ[i+2] = Q[2] * 2.0;
        dQ[i+3] = Q[3] * 2.0;
    }
    dQ[dimx] = dQ[nx-2] = (R[0] + Rd[0]) * 2.0;
    dQ[dimx + 1] = dQ[nx-1] = (R[1] + Rd[1]) * 2.0;
    for (int i=dimx+2; i<nx-2; i+=2)
    {
        dQ[i] = 2 * (R[0] + 2 * Rd[0]);
        dQ[i+1] = 2 * (R[1] + 2 * Rd[1]);
    }
    for (int i=nx; i<nnzQ; i+=2)
    {
        dQ[i] = -Rd[0] * 2.0;
        dQ[i+1] = -Rd[1] * 2.0;
    }
    hessian.resize(nx, nx);
    Eigen::MatrixXd QQ(nx, nx);
    for (int i=0; i<nx; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
    }
    for (int i=nx; i<nnzQ; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
        hessian.insert(jcolQ[i], irowQ[i]) = dQ[i];
    }

    // equality constraints
    MPCState temp = xbar[delay_num];
    getLinearModel(temp);
    int my = dimx;
    double b[my];
    const int nnzA = 14 * (T-delay_num) - 8;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];
    Eigen::Vector4d temp_vec(temp.x, temp.y, temp.v, temp.theta);
    Eigen::Vector4d temp_b = A*temp_vec + C;
    
    for (int i=0; i<dimx; i++)
    {
        irowA[i] = jcolA[i] = i;
        dA[i] = 1;
    }
    b[0] = temp_b[0];
    b[1] = temp_b[1];
    b[2] = temp_b[2];
    b[3] = temp_b[3];
    irowA[dimx] = 2;
    jcolA[dimx] = dimx;
    dA[dimx] = -B(2, 0);
    irowA[dimx+1] = 3;
    jcolA[dimx+1] = dimx+1;
    dA[dimx+1] = -B(3, 1);
    int ABidx = 10*(T-delay_num) - 10;
    int ABbegin = dimx+2;
    for (int i=0, j=1; i<ABidx; i+=10, j++)
    {
        getLinearModel(xbar[j+delay_num]);
        for (int k=0; k<4; k++)
        {
            b[4*j+k] = C[k];
            irowA[ABbegin + i + k] = 4*j + k;
            jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 4;
            dA[ABbegin + i + k] = -A(k, k);
        }
        irowA[ABbegin + i + 4] = 4*j;
        jcolA[ABbegin + i + 4] = 4*j - 2;
        dA[ABbegin + i + 4] = -A(0, 2);
        
        irowA[ABbegin + i + 5] = 4*j;
        jcolA[ABbegin + i + 5] = 4*j - 1;
        dA[ABbegin + i + 5] = -A(0, 3);

        irowA[ABbegin + i + 6] = 4*j + 1;
        jcolA[ABbegin + i + 6] = 4*j - 2;
        dA[ABbegin + i + 6] = -A(1, 2);

        irowA[ABbegin + i + 7] = 4*j + 1;
        jcolA[ABbegin + i + 7] = 4*j - 1;
        dA[ABbegin + i + 7] = -A(1, 3);
        
        irowA[ABbegin + i + 8] = 4*j +2;
        jcolA[ABbegin + i + 8] = dimx + 2*j;
        dA[ABbegin + i + 8] = -B(2, 0);
        
        irowA[ABbegin + i + 9] = 4*j + 3;
        jcolA[ABbegin + i + 9] = dimx + 2*j + 1;
        dA[ABbegin + i + 9] = -B(3, 1);
    }

    // iequality constraints
    const int mz  = (T-delay_num) - 1;
    const int nnzC = dimu - 2;
    int   irowC[nnzC];
    int   jcolC[nnzC];
    double   dC[nnzC];
    for (int i=0, j=0; i<mz; i++, j+=2)
    {
        irowC[j] = i;
        irowC[j+1] = i;
        jcolC[j] = dimx + 1 + j;
        jcolC[j+1] = jcolC[j] +2;
        dC[j] = -1.0;
        dC[j+1] = 1.0;
    }

    // xlimits and all
    int mx = 3*(T-delay_num);
    int nc = mx+my+mz;
    lowerBound.resize(nc);
    upperBound.resize(nc);
    linearMatrix.resize(nc, nx);
    for (int i=0, j=0, k=0; i<mx; i+=3, j+=4, k+=2)
    {
        lowerBound[i] = -max_speed;
        lowerBound[i+1] = -max_accel;
        lowerBound[i+2] = -max_omega;
        upperBound[i] = max_speed;
        upperBound[i+1] = max_accel;
        upperBound[i+2] = max_omega;
        linearMatrix.insert(i, j+2) = 1;
        linearMatrix.insert(i+1, dimx+k) = 1;
        linearMatrix.insert(i+2, dimx+k+1) = 1;
    }
    for (int i=0; i<nnzA; i++)
    {
        linearMatrix.insert(irowA[i]+mx, jcolA[i]) = dA[i];
    }
    for (int i=0; i<my; i++)
    {
        lowerBound[mx+i] = upperBound[mx+i] = b[i];
    }
    for (int i=0; i<nnzC; i++)
    {
        linearMatrix.insert(irowC[i]+mx+my, jcolC[i]) = dC[i];
    }
    for (int i=0; i<mz; i++)
    {
        lowerBound[mx+my+i] = -max_comega;
        upperBound[mx+my+i] = max_comega;
    }

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-6);
    solver.settings()->setMaxIteration(30000);
    solver.settings()->setRelativeTolerance(1e-6);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);
    if(!solver.data()->setHessianMatrix(hessian)) return;
    if(!solver.data()->setGradient(gradient)) return;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return;
    if(!solver.data()->setLowerBound(lowerBound)) return;
    if(!solver.data()->setUpperBound(upperBound)) return;

    // instantiate the solver
    if(!solver.initSolver()) return;

    // controller input and QPSolution vector
    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;

    // get the controller input
    QPSolution = solver.getSolution();
    for (int i=0; i<delay_num; i++)
    {
        output(0, i) = output_buff[i][0];
        output(1, i) = output_buff[i][1];
    }
    for (int i=0, j=0; i<dimu; i+=2, j++)
    {
        output(0, j+delay_num) = QPSolution[dimx+i];
        output(1, j+delay_num) = QPSolution[dimx+i+1];
    }
}

void MPC::solveMPCV(void)
{
    const int dimx = 3 * (T - delay_num);
    const int dimu = 2 * (T - delay_num);
    const int nx = dimx + dimu;

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // first-order
    for (int i=0, j=delay_num, k=0; i<dimx; i+=3, j++, k+=2)
    {
        gradient[i] = -2 * Q[0] * xref(0, j);
        gradient[i+1] = -2 * Q[1] * xref(1, j);
        gradient[i+2] = -2 * Q[3] * xref(3, j);
        gradient[dimx+k] = -2 * Q[2] * dref(0, j);
    }

    // second-order
    const int nnzQ = nx + dimu - 2;
    int irowQ[nnzQ];
    int jcolQ[nnzQ];
    double dQ[nnzQ];
    for (int i=0; i<nx; i++)
    {
        irowQ[i] = jcolQ[i] = i;
    }
    for (int i=nx; i<nnzQ; i++)
    {
        irowQ[i] = i - dimu + 2;
        jcolQ[i] = i - dimu;
    }
    for (int i=0; i<dimx; i+=3)
    {
        dQ[i] = Q[0] * 2.0;
        dQ[i+1] = Q[1] * 2.0;
        dQ[i+2] = Q[3] * 2.0;
    }
    dQ[dimx] = dQ[nx-2] = (R[0] + Rd[0] + Q[2]) * 2.0;
    dQ[dimx + 1] = dQ[nx-1] = (R[1] + Rd[1]) * 2.0;
    for (int i=dimx+2; i<nx-2; i+=2)
    {
        dQ[i] = 2 * (R[0] + 2 * Rd[0] + Q[2]);
        dQ[i+1] = 2 * (R[1] + 2 * Rd[1]);
    }
    for (int i=nx; i<nnzQ; i+=2)
    {
        dQ[i] = -Rd[0] * 2.0;
        dQ[i+1] = -Rd[1] * 2.0;
    }
    hessian.resize(nx, nx);
    Eigen::MatrixXd QQ(nx, nx);
    for (int i=0; i<nx; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
    }
    for (int i=nx; i<nnzQ; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
        hessian.insert(jcolQ[i], irowQ[i]) = dQ[i];
    }

    // equality constraints
    MPCState temp = xbar[delay_num];
    getLinearModel(temp);
    int my = dimx;
    double b[my];
    const int nnzA = 11 * (T-delay_num) - 5;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];
    Eigen::Vector3d temp_vec(temp.x, temp.y, temp.theta);
    Eigen::Vector3d temp_b = A*temp_vec + C;
    
    for (int i=0; i<dimx; i++)
    {
        irowA[i] = jcolA[i] = i;
        dA[i] = 1;
    }
    b[0] = temp_b[0];
    b[1] = temp_b[1];
    b[2] = temp_b[2];
    irowA[dimx] = 0;
    jcolA[dimx] = dimx;
    dA[dimx] = -B(0, 0);
    irowA[dimx+1] = 1;
    jcolA[dimx+1] = dimx;
    dA[dimx+1] = -B(1, 0);
    irowA[dimx+2] = 2;
    jcolA[dimx+2] = dimx+1;
    dA[dimx+2] = -B(2, 1);
    int ABidx = 8*(T-delay_num) - 8;
    int ABbegin = dimx+3;
    for (int i=0, j=1; i<ABidx; i+=8, j++)
    {
        getLinearModel(xbar[j+delay_num]);
        for (int k=0; k<3; k++)
        {
            b[3*j+k] = C[k];
            irowA[ABbegin + i + k] = 3*j + k;
            jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 3;
            dA[ABbegin + i + k] = -A(k, k);
        }
        irowA[ABbegin + i + 3] = 3*j;
        jcolA[ABbegin + i + 3] = 3*j - 1;
        dA[ABbegin + i + 3] = -A(0, 2);

        irowA[ABbegin + i + 4] = 3*j + 1;
        jcolA[ABbegin + i + 4] = 3*j - 1;
        dA[ABbegin + i + 4] = -A(1, 2);
        
        irowA[ABbegin + i + 5] = 3*j;
        jcolA[ABbegin + i + 5] = dimx + 2*j;
        dA[ABbegin + i + 5] = -B(0, 0);
        
        irowA[ABbegin + i + 6] = 3*j + 1;
        jcolA[ABbegin + i + 6] = dimx + 2*j;
        dA[ABbegin + i + 6] = -B(1, 0);
        
        irowA[ABbegin + i + 7] = 3*j + 2;
        jcolA[ABbegin + i + 7] = dimx + 2*j + 1;
        dA[ABbegin + i + 7] = -B(2, 1);
    }

    // iequality constraints
    const int mz  = 2 * (T-delay_num) - 2;
    const int nnzC = 2 * dimu - 4;
    int   irowC[nnzC];
    int   jcolC[nnzC];
    double   dC[nnzC];
    for (int i=0, k=0; i<mz; i+=2, k+=4)
    {
        irowC[k] = i;
        jcolC[k] = dimx  + i;
        dC[k] = -1.0;

        irowC[k+1] = i;
        jcolC[k+1] = jcolC[k] +2;
        dC[k+1] = 1.0;

        irowC[k+2] = i + 1;
        jcolC[k+2] = dimx + 1 + i;
        dC[k+2] = -1.0;

        irowC[k+3] = i + 1;
        jcolC[k+3] = jcolC[k+2] +2;
        dC[k+3] = 1.0;
    }

    // xlimits and all
    int mx = dimu;
    int nc = mx+my+mz;
    lowerBound.resize(nc);
    upperBound.resize(nc);
    linearMatrix.resize(nc, nx);
    for (int i=0; i<mx; i+=2)
    {
        lowerBound[i] = -max_speed;
        lowerBound[i+1] = -max_omega;
        upperBound[i] = max_speed;
        upperBound[i+1] = max_omega;
        linearMatrix.insert(i, dimx+i) = 1;
        linearMatrix.insert(i+1, dimx+i+1) = 1;
    }

    for (int i=0; i<nnzA; i++)
    {
        linearMatrix.insert(irowA[i]+mx, jcolA[i]) = dA[i];
    }

    for (int i=0; i<my; i++)
    {
        lowerBound[mx+i] = upperBound[mx+i] = b[i];
    }

    for (int i=0; i<nnzC; i++)
    {
        linearMatrix.insert(irowC[i]+mx+my, jcolC[i]) = dC[i];
    }

    for (int i=0; i<mz; i+=2)
    {
        lowerBound[mx+my+i] = -max_cv;
        upperBound[mx+my+i] = max_cv;
        lowerBound[mx+my+i+1] = -max_comega;
        upperBound[mx+my+i+1] = max_comega;
    }

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-6);
    solver.settings()->setMaxIteration(30000);
    solver.settings()->setRelativeTolerance(1e-6);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);
    if(!solver.data()->setHessianMatrix(hessian)) return;
    if(!solver.data()->setGradient(gradient)) return;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return;
    if(!solver.data()->setLowerBound(lowerBound)) return;
    if(!solver.data()->setUpperBound(upperBound)) return;

    // instantiate the solver
    if(!solver.initSolver()) return;

    // controller input and QPSolution vector
    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;

    // get the controller input
    QPSolution = solver.getSolution();
    // ROS_INFO("Solution: v0=%f     omega0=%f", QPSolution[dimx], QPSolution[dimx+1]);
    for (int i=0; i<delay_num; i++)
    {
        output(0, i) = output_buff[i][0];
        output(1, i) = output_buff[i][1];
    }
    for (int i=0, j=0; i<dimu; i+=2, j++)
    {
        output(0, j+delay_num) = QPSolution[dimx+i];
        output(1, j+delay_num) = QPSolution[dimx+i+1];
    }
}

void MPC::getCmd(void)
{
    int iter;
    ros::Time begin = ros::Time::now();
    for (iter=0; iter<max_iter; iter++)
    {
        predictMotion();
        last_output = output;
        if (control_a)
            solveMPCA();
        else
            solveMPCV();
        double du = 0;
        for (int i=0; i<output.cols(); i++)
        {
            du = du + fabs(output(0, i) - last_output(0, i))+ fabs(output(1, i) - last_output(1, i));
        }
        // break;
        if (du <= du_th || (ros::Time::now()-begin).toSec()>0.01)
        {
            break;
        }
    }
    if (iter == max_iter)
    {
        ROS_WARN("MPC Iterative is max iter");
    }

    predictMotion(xopt);
    if (in_bk)
    {
        // drawRefPath();
        drawBkPath();
    }
    else
        drawRefPath();
    drawPredictPath(xopt);
    if (control_a)
    {
        cmd.speed = now_state.v + dt * output(0, delay_num);
    }
    else
        cmd.speed = output(0, delay_num);
    cmd.omega = output(1, delay_num);
    if (delay_num>0)
    {
        output_buff.erase(output_buff.begin());
        output_buff.push_back(Eigen::Vector2d(output(0, delay_num),output(1, delay_num)));
    }
}
