#pragma once

#include "mpc/Polynome.h"
#include "mpc_utils/minco.hpp"
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <iostream>

template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    // AERROR << "input time difference is too small";
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

inline double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

class TrajPoint
{
public:
    double x = 0;
    double y = 0;
    double v = 0; 
    double a = 0;
    double z = 0;
    double theta = 0;
    double w = 0;
};

class TrajAnalyzer{
public:
    mpc_utils::MinJerkOpt jerk_opt;
    mpc_utils::Trajectory minco_traj;
    ros::Time start_time;
    double traj_duration;

public:

    std::vector<mpc_utils::Trajectory> way_minco_traj;
    std::vector<std::vector<TrajPoint>> vis_trajs;
    std::vector<std::vector<std::pair<double, double>>> way_liuming_traj;
    bool at_goal = false;
    size_t track_seg = 0;

    TrajAnalyzer() {}

    Eigen::Vector3d nextInitPointMinco()
    {
      if (track_seg < way_minco_traj.size())
      {
        mpc_utils::Trajectory p = way_minco_traj[track_seg];
        Eigen::Vector3d po = p.getPos(0);
        return po;
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    }

    Eigen::Vector2d nextInitPointLiuming()
    {
      if (track_seg < way_liuming_traj.size())
      {
        return Eigen::Vector2d(way_liuming_traj[track_seg][0].first, way_liuming_traj[track_seg][1].second);
      }
      else
      {
        return Eigen::Vector2d::Zero();
      }
    }

    std::vector<std::pair<double, double>> beginNextTrajLiuming()
    {
      if (track_seg < way_liuming_traj.size())
      {
        return way_liuming_traj[track_seg++];
      }
    }

    void beginNextTrajMinco()
    {
      if (track_seg < way_minco_traj.size())
      {
        minco_traj = way_minco_traj[track_seg++];
        traj_duration = minco_traj.getTotalDuration();
        ros::Duration d(5.0);
        d.sleep();
        start_time = ros::Time::now();
        at_goal = false;
      }
    }

    void setTraj(const char* file_name)
    {      
      ifstream file;
      string find_data("data");
      string file_name_s(file_name);
      size_t f_index = file_name_s.find(find_data) + 5;
      file.open(file_name, ios::in);

      string data_line;
      int n = 0;

      if (file_name[f_index]=='t')
      {
        while(getline(file, data_line,'\n'))
        {
          n++;
          string data;
          vector<string> datas;
          stringstream line(data_line);
          while(line >> data)
            datas.push_back(data);
          
          vector<double> T;
          vector<Eigen::Matrix<double, 3, 6>> cMats;
          Eigen::Matrix<double, 3, 6> cMat;
          int traj_pieces  = stoi( datas[0] );
          int index = 1;

          for(int i = 0 ; i < traj_pieces ; i++) {
            T.push_back( stod(datas[index++] ));
            for( int a = 0 ; a < 3 ; a++ )
            {
              for(int b = 0 ; b < 6 ; b++)
              {
                cMat(a,b) = stod( datas[ index++ ] );
              }
            }
            cMats.push_back( cMat );
          }
          mpc_utils::Trajectory traj;
          traj.reserve(traj_pieces);

          for (int i = 0; i < traj_pieces; i++) {
              traj.emplace_back(T[i], cMats[i] );
          }
          way_minco_traj.push_back(traj);
          std::vector<TrajPoint> now_vis_traj;
          for(double t = 0; t < traj.getTotalDuration(); t += 0.05)
          {
            TrajPoint tp;
            Eigen::Vector3d tp_vec = traj.getPos(t);
            tp.x = tp_vec[0];
            tp.y = tp_vec[1];
            tp.z = tp_vec[2];
            now_vis_traj.push_back(tp);
          }
          vis_trajs.push_back(now_vis_traj);
        }  
      }
      else if (file_name[f_index]=='l')
      {
        while(getline(file, data_line,'\n'))
        {
          n++;
          string data;
          vector<string> datas;
          stringstream line(data_line);
          while(line >> data)
            datas.push_back(data);
          
          vector<pair<double, double>> traj;
          std::vector<TrajPoint> now_vis_traj;
          for (size_t i=0; i<datas.size(); i+=2)
          {
            TrajPoint tp;
            tp.x = stod(datas[i]);
            tp.y = stod(datas[i+1]);
            tp.z = 2.0;
            traj.push_back(make_pair<double, double>(stod(datas[i]), stod(datas[i+1])));
            now_vis_traj.push_back(tp);
          }
          way_liuming_traj.push_back(traj);
          vis_trajs.push_back(now_vis_traj);
        }  
      }
      std::cout<<"We have "<<n<<" traj!"<<std::endl;
      at_goal = true;
      file.close();
    }

    void setTraj(mpc::PolynomeConstPtr msg)
    {
      start_time = msg->start_time;

      Eigen::MatrixXd posP(3, msg->pos_pts.size() - 2);
      Eigen::VectorXd T(msg->t_pts.size());
      Eigen::MatrixXd initS, tailS;

      for (int i = 1; i < (int)msg->pos_pts.size() - 1; i++)
      {
        posP(0, i - 1) = msg->pos_pts[i].x;
        posP(1, i - 1) = msg->pos_pts[i].y;
        posP(2, i - 1) = msg->pos_pts[i].z;
      }
      for (int i = 0; i < (int)msg->t_pts.size(); i++)
      {
        T(i) = msg->t_pts[i];
      }

      initS.setZero(3, 3);
      tailS.setZero(3, 3);
      initS.col(0) = Eigen::Vector3d(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
      initS.col(1) = Eigen::Vector3d(msg->init_v.x, msg->init_v.y, msg->init_v.z);
      initS.col(2) = Eigen::Vector3d(msg->init_a.x, msg->init_a.y, msg->init_a.z);
      tailS.col(0) = Eigen::Vector3d(msg->pos_pts.back().x, msg->pos_pts.back().y, msg->pos_pts.back().z);
      tailS.col(1) = Eigen::Vector3d::Zero();
      tailS.col(2) = Eigen::Vector3d::Zero();
      jerk_opt.reset(initS, msg->pos_pts.size() - 1);
      jerk_opt.generate(posP, tailS, T);
      minco_traj = jerk_opt.getTraj();
      traj_duration = minco_traj.getTotalDuration();
      start_time = ros::Time::now();
    }

    std::vector<TrajPoint> getRefPoints(const int T, double dt)
    {
      std::vector<TrajPoint> P;
      P.clear();
      TrajPoint tp;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - start_time).toSec();
      int j=0;

      if (t_cur > traj_duration + 1.0)
      {
        at_goal = true;
        return P;
      }
      else
      {
        at_goal = false;
      }

      for (double t=t_cur+dt; j<T; j++, t+=dt)
      {
        double temp = t;
        if (temp <= traj_duration)
        {
          Eigen::Vector3d po = minco_traj.getPos(temp);
          Eigen::Vector3d pr = minco_traj.getPos(temp-dt);
          Eigen::Vector2d v_xy = minco_traj.getVel(temp).head(2);

          tp.v = v_xy.norm();
          // tp.v = minco_traj.getVel(temp).norm();
          tp.x = po[0];
          tp.y = po[1];
          tp.z = po[2];
          tp.a = minco_traj.getAcc(temp).norm();
          tp.theta = atan2(po[1]-pr[1], po[0]-pr[0]);
          tp.w = 0;
          P.push_back(tp);
        }
        else
        {
          Eigen::Vector3d po = minco_traj.getPos(traj_duration);
          Eigen::Vector3d pr = minco_traj.getPos(traj_duration-dt);
          Eigen::Vector2d v_xy = minco_traj.getVel(temp).head(2);

          tp.v = v_xy.norm();
          // tp.v = minco_traj.getVel(traj_duration).norm();
          tp.x = po[0];
          tp.y = po[1];
          tp.z = po[2];
          tp.a = minco_traj.getAcc(traj_duration).norm();
          tp.theta = atan2(po[1]-pr[1], po[0]-pr[0]);
          tp.w = 0;
          P.push_back(tp);
        }
      }

      return P;
    }

    TrajPoint getErrState(void)
    {
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - start_time).toSec();
      TrajPoint tp;

      if (t_cur <= traj_duration)
      {
        Eigen::Vector3d po = minco_traj.getPos(t_cur);

        tp.x = po[0];
        tp.y = po[1];
        tp.w = 0;
        return tp;
      }
      else
      {
        Eigen::Vector3d po = minco_traj.getPos(traj_duration);
        
        tp.x = po[0];
        tp.y = po[1];
        return tp;
      }

      return tp;
    }

   
    ~TrajAnalyzer() {}
};