#pragma once
#include <map_manager/PCSmap_manager.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <chrono>
#include <thread>

#include "minco.hpp"

class TrajOpt {

  public:
      //ros
      ros::NodeHandle nh;
      ros::Publisher debug_pub, debug_wp_pub;

      PCSmapManager::Ptr environment;

      bool pause_debug = true;
      // # pieces and # key points
      int N, K, dim_t, dim_p;

      // weight for time regularization term
      double rhoT;

      // collision avoiding and dynamics paramters
      double pok, vmax, amax, vmax_h , amax_h;
      double rhoP, rhoV, rhoA;
      double rhoTracking_, rhosVisibility_;
      double clearance_d_, tolerance_d_, theta_clearance_;
      // SE3 dynamic limitation parameters
      double thrust_max_, thrust_min_;
      double omega_max_, omega_yaw_max_;
      
      // corridor
      std::vector<Eigen::MatrixXd> cfgVs_;
      std::vector<Eigen::MatrixXd> cfgHs_;

      // Minimum Jerk Optimizer
      minco::MinJerkOpt jerkOpt;

      // col(0) is P of (x,y,z), col(1) is V .. ， col(2) is A
      Eigen::MatrixXd initS;  
      Eigen::MatrixXd finalS;

      // weight for each vertex
      Eigen::VectorXd p;

      // duration of each piece of the trajectory
      Eigen::VectorXd t;
      Eigen::VectorXd x;
      //double* x;
      double sum_T;

      std::vector<Eigen::Vector3d> tracking_ps_;
      std::vector<Eigen::Vector3d> tracking_visible_ps_;
      std::vector<double> tracking_thetas_;
      double tracking_dur_;
      double tracking_dist_;
      double tracking_dt_;

      // polyH utils
      bool extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                     std::vector<Eigen::MatrixXd>& vPs) const;

      void drawDebug(Trajectory end_path , Eigen::Map<const Eigen::MatrixXd> P);
      void drawDebugWp(std::vector<Eigen::Vector3d> end_path);
      //void deleteX(){delete[] x;}

  public:
      TrajOpt() {}
      ~TrajOpt() {}

      void setParam(ros::NodeHandle& nh)
      {
        this->nh = nh;
        nh.param("optimization/K", K, 8);
        nh.param("optimization/pok", pok, 0.3);
        nh.param("optimization/vmax", vmax, 3.0);
        nh.param("optimization/amax", amax, 10.0);
        nh.param("optimization/vmaxz", vmax_h, 0.2);
        nh.param("optimization/amaxz", amax_h, 0.4);
        nh.param("optimization/rhoT", rhoT, 1000.0);
        nh.param("optimization/rhoP", rhoP, 1000000.0);
        nh.param("optimization/rhoV", rhoV, 1000.0);
        nh.param("optimization/rhoA", rhoA, 1000.0);
        nh.param("optimization/pause_debug", pause_debug, false);
        debug_pub = nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_path", 10);
        debug_wp_pub = nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_path_wp", 10);

      }
      void setEnvironment(const PCSmapManager::Ptr& mapPtr)
      {
          environment = mapPtr;
      }
      bool generate_traj(const Eigen::MatrixXd& initState,
                         const Eigen::MatrixXd& finalState,
                         const std::vector<Eigen::Vector3d>& Q,
                         const int N,
                         Trajectory& traj,
                         bool keep_result);

      void addTimeIntPenalty(double& cost);
      bool grad_cost_p(const Eigen::Vector3d& p,
                       Eigen::Vector3d& gradp,
                       double& costp);
      bool grad_cost_v(const Eigen::Vector3d& v,
                       Eigen::Vector3d& gradv,
                       double& costv);
      bool grad_cost_a(const Eigen::Vector3d& a,
                       Eigen::Vector3d& grada,
                       double& costa);
                       
      bool grad_cost_shape(const Eigen::Vector3d& p,const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradp,
                          double& costp,Eigen::Vector3d& gradv,double& costv);
  public:
    typedef shared_ptr<TrajOpt> Ptr;

};
