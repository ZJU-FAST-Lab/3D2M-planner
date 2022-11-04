#include "planner_manager/planner_manager.h"
#include "mpc/Polynome.h"

#include <fstream>
#include <random>
#include <time.h>

#define COORDS_FILE "/home/lantern/ROS_workspace/diablo-planner/data/corrds.txt"
#define TRAJS_FILE  "/home/lantern/ROS_workspace/diablo-planner/data/trajs.txt"
#define TIMES_FILE  "/home/lantern/ROS_workspace/diablo-planner/data/times.txt"
#define CURVE_FILE  "/home/lantern/ROS_workspace/diablo-planner/data/curve.txt"

#define NORMAL
// #define BENCHMARK100
// #define BENCHMARKTIME


ofstream coords_writer, trajs_writer;

void PlannerManager::init(ros::NodeHandle& nh)
{
  pcsmap_manager.reset(new PCSmapManager);
  pcsmap_manager -> init(nh);

  astar_searcher.reset(new AstarPathSearcher);
  astar_searcher -> init(nh);

  minco_traj_optimizer.reset(new TrajOpt);
  minco_traj_optimizer -> setParam(nh);
  minco_traj_optimizer -> setEnvironment(pcsmap_manager);

  nh.param("traj_parlength", traj_parlength, 2.0);
  nh.param("truncation_dis", truncation_dis, 2.0);


  (pcsmap_manager->offground_threshold) = pcsmap_manager->occupancy_resolution;
  //(pcsmap_manager->offground_threshold) = 0.05;

  (pcsmap_manager->travelcost_map) -> truncation_dis = truncation_dis;


  
  odom_sub      = nh.subscribe("odom", 1, &PlannerManager::odomRcvCallBack, this);
  target_sub    = nh.subscribe("/goal",1, &PlannerManager::targetRcvCallBack, this);
  target_sub2    = nh.subscribe("/anchor_3D_goal",10, &PlannerManager::target2RcvCallBack, this );
  benchmark_run_sub  = nh.subscribe("/run_benchmark_traj", 1 , &PlannerManager::rcvBenchmarkRun, this); 

  path_vis_pub  = nh.advertise<visualization_msgs::Marker>("path_vis", 10);
  traj_vis_pub  = nh.advertise<visualization_msgs::Marker>("traj_vis", 10000);
  point_vis_pub = nh.advertise<visualization_msgs::Marker>("points_vis", 10);
  flash_pub     = nh.advertise<geometry_msgs::Pose>("/diablo_flash", 10);

  cmd_geo_pub   = nh.advertise<geometry_msgs::Pose>("cmd_geo", 200);

  traj_pub      = nh.advertise<mpc::Polynome>("trajectory",3);

  rcvmap_signal_sub = nh.subscribe("/rcvmap_signal",1, &PlannerManager::mapRcvCallBack, this);

  has_odom = false;
}


void PlannerManager::pubFlash(Vector3d pos, Vector3d vel)
{
  geometry_msgs::Pose msg;
  msg.position.x = pos(0);
  msg.position.y = pos(1);
  msg.position.z = pos(2) + 0.1;

  Quaterniond quaternion;
  quaternion = AngleAxisd(0, Eigen::Vector3d::UnitZ()) * 
                 AngleAxisd(atan2( -vel(0), vel(2) ), Eigen::Vector3d::UnitY()) * 
                 AngleAxisd(0, Eigen::Vector3d::UnitX());

  msg.orientation.x = quaternion.x();
  msg.orientation.y = quaternion.y();
  msg.orientation.z = quaternion.z();
  msg.orientation.w = quaternion.w();

  flash_pub.publish(msg);
}

void PlannerManager::odomRcvCallBack(const nav_msgs::Odometry& msg)
{
  recent_odom = msg;
  has_odom = true;
}

void PlannerManager::mapRcvCallBack(const std_msgs::Empty& msg)
{
    astar_searcher -> initGridMap(pcsmap_manager);
    cout<<"init map A*-"<<endl;
}


//这两个函数一起可以读取文件中某一特定行，行数line从1开始，Readline返回值为string类型
int CountLines(string filename)
{
    ifstream ReadFile;
    int n=0;
    string tmp;
    ReadFile.open(filename.c_str(),ios::in);//ios::in 表示以只读的方式读取文件
    if(ReadFile.fail())//文件打开失败:返回0
    {
        return 0;
    }
    else//文件存在
    {
        while(getline(ReadFile,tmp,'\n'))
        {
            n++;
        }
        ReadFile.close();
        return n;
    }
}
 
string ReadLine(string filename,int line)
{
    int lines,i=0;
    string temp;
    fstream file;
    file.open(filename.c_str(), ios::in);
    lines = CountLines(filename);
 
    if(line<=0)
    {
        return "Error 1: 行数错误，不能为0或负数。";
    }
    if(file.fail())
    {
        return "Error 2: 文件不存在。";
    }
    if(line > lines)
    {
        return "Error 3: 行数超出文件长度。";
    }
    while(getline(file,temp) && i < line-1)
    {
        i++;
    }
    file.close();
    return temp;
}

void PlannerManager::rcvBenchmarkRun(const std_msgs::Int16 msg)
{
  int item = msg.data;
  cout<<"[Planner Manager] 准备执行第 " << item <<" 条benchmark轨迹 "<<endl;
  
  string data_line = ReadLine(COORDS_FILE, item);
  string data;
  vector<string> datas;
  stringstream line(data_line);
  while(line >> data)
        datas.push_back(data);
    //输出data
  for(int i = 0; i < datas.size() ; i++)
  {
        cout<<datas[i]<<endl;
  }
  Vector3d begin_pos, begin_vel;
  vector<double> T;
  vector<Eigen::Matrix<double, 3, 6>> cMats;
  Eigen::Matrix<double, 3, 6> cMat;

  begin_pos(0) = stod( datas[0] );
  begin_pos(1) = stod( datas[1] );
  begin_pos(2) = stod( datas[2] );

  data_line = ReadLine(TRAJS_FILE, item);
  stringstream line2(data_line);
  datas.clear();
  while(line2 >> data) {
        datas.push_back(data);
  }

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


  Trajectory traj;
  traj.reserve(traj_pieces);

  for (int i = 0; i < traj_pieces; i++) {
      traj.emplace_back(T[i], cMats[i] );
  }
  recent_traj = traj;
  begin_vel = recent_traj.getVel(0.4);

  renderTraj(recent_traj);
  pubFlash( begin_pos , begin_vel);
  pubStop();

  ros::Duration(3).sleep();
  publishTraj();
}

void PlannerManager::pubStop()
{
  geometry_msgs::Pose cmd_geo;
  cmd_geo.position.x = 0;
  cmd_geo.position.y = 0;

  cmd_geo_pub.publish(cmd_geo);
}

void PlannerManager::targetRcvCallBack(const geometry_msgs::PoseStamped& msg)
{

    Vector3d begin_pos;
    Vector3d target_pos;
    
    Vector3d begin_vel;
    Vector3d target_vel;

    int ret_value;


#ifdef NORMAL

    target_pos(0) = msg.pose.position.x;
    target_pos(1) = msg.pose.position.y;
    target_pos(2) = msg.pose.position.z;

    Quaterniond    q(	msg.pose.orientation.w,
							    	  msg.pose.orientation.x,
							    	  msg.pose.orientation.y,
							    	  msg.pose.orientation.z  );

		Matrix3d       R(q);
		double target_yaw 	= atan2(R.col(0)[1],R.col(0)[0]);    

    target_vel(0) = cos(target_yaw);
    target_vel(1) = sin(target_yaw);
    target_vel(2) = 0;

    target_vel.normalize();
    target_vel *= 0.01;

    //6.39, -9.77, 21.7

    //cout<<"tr_cost = " << (pcsmap_manager->travelcost_map) -> getSDFValue(target_pos)<<endl;
    //cout<<"tr_grad = " << (pcsmap_manager->travelcost_map) -> getSDFGrad(target_pos)<<endl;
    double sdf_cost = (pcsmap_manager->occupancy_map) -> getSDFValue(target_pos);
    cout<<"sdf_cost = " << sdf_cost <<endl;
    cout<<"sdf_grad = " << (pcsmap_manager->occupancy_map) -> getSDFGrad(target_pos)<<endl;
    //Vector3d up_normal = Vector3d(0,0,1);
    //cout<<"plane_cost = " << 1 - up_normal.transpose() * (pcsmap_manager->travelcost_map) -> getTravelPN(target_pos)<<endl;

    pcsmap_manager -> dropPoint(target_pos);
    cout<<"target = " <<target_pos<<endl;

    if(has_odom)
    {
      begin_pos(0) = recent_odom.pose.pose.position.x;
      begin_pos(1) = recent_odom.pose.pose.position.y;
      begin_pos(2) = recent_odom.pose.pose.position.z + 1.0;
      // begin_pos(2) = recent_odom.pose.pose.position.z;

      begin_vel(0) = 0;
      begin_vel(1) = 0;
      begin_vel(2) = 0;

      // begin_vel(0) = recent_odom.twist.twist.linear.x;
      // begin_vel(1) = recent_odom.twist.twist.linear.y;
      // begin_vel(2) = 0;

      // begin_pos(0) = -4.37;
      // begin_pos(1) = -13.1;
      // begin_pos(2) = 11.61;
      pcsmap_manager -> dropPoint(begin_pos);
      ros::Time before_planning = ros::Time::now();
      if( generatePath( begin_pos, target_pos ) )
      {
        generateTraj(recent_path, begin_vel, target_vel,ret_value);
      }

      ros::Time after_planning = ros::Time::now();
      std::cout<<"[Planner Manager] total cost = " << 1000*(after_planning.toSec() - before_planning.toSec())<<"ms"<<std::endl;
    }

#endif

#ifdef BENCHMARK100


    coords_writer.open(COORDS_FILE);
    trajs_writer.open(TRAJS_FILE);

    srand(time(0));
    int times = 100;
    int total_times = 0;
    int x_size  = floor(10*( pcsmap_manager -> boundary_xyzmax(0) - pcsmap_manager -> boundary_xyzmin(0) ));
    int y_size  = floor(10*( pcsmap_manager -> boundary_xyzmax(1) - pcsmap_manager -> boundary_xyzmin(1) ));
    int z_value = floor( (pcsmap_manager -> boundary_xyzmax(2)) );
    int x_begin = 10 * ceil(pcsmap_manager -> boundary_xyzmin(0));
    int y_begin = 10 * ceil(pcsmap_manager -> boundary_xyzmin(1));
    while( times-- ){
      
      total_times++;
      if(total_times > 1000){
        std::cout<<" [benchmark100] 重试次数达到最大值，终止。 "<<std::endl;
        break;
      }
      std::cout<<" [benchmark100] 执行第 "<< 101 - times <<" 次 benchmark "<<std::endl;
      begin_pos(0) = 0.1 * ( rand() % x_size + x_begin );
      begin_pos(1) = 0.1 * ( rand() % y_size + y_begin );
      begin_pos(2) = z_value;

      // begin_pos(0) = msg.pose.position.x;
      // begin_pos(1) = msg.pose.position.y;
      // begin_pos(2) = msg.pose.position.z;

      begin_vel(0) = 0;
      begin_vel(1) = 0;
      begin_vel(2) = 0;

      target_pos(0) = 0.1 * ( rand() % x_size + x_begin );
      target_pos(1) = 0.1 * ( rand() % y_size + y_begin );
      target_pos(2) = z_value;

      target_vel(0) = 0;
      target_vel(1) = 0;
      target_vel(2) = 0;

      

      pcsmap_manager -> dropPoint(begin_pos);
      pcsmap_manager -> dropPoint(target_pos);

      if( (begin_pos - target_pos).norm() < 5 ){ 
        std::cout<<" [benchmark100] 起点距离终点太近，重新执行。 "<<std::endl;
        times++; continue; 
      }

      vector<Vector3d> pts;
      pts.push_back( begin_pos );
      pts.push_back( target_pos );

      renderPoints(pts, Vector3d(1,1,0) , 2.0, 1);



      if( generatePath( begin_pos, target_pos ) )
      {
        renderPath(recent_path);
        generateTraj(recent_path, begin_vel, target_vel, ret_value);
        if( ret_value < 0){
          std::cout<<" [benchmark100] 优化失败，重新执行。 "<<std::endl;
          times++;
          continue;
        }
        else{
          renderTraj(recent_traj);
          std::cout<<" [benchmark100] 成功，写入文件。 "<<std::endl;
          coords_writer.precision(15); 
          trajs_writer.precision(15); 
          coords_writer << begin_pos(0) << " " << begin_pos(1) <<" " << begin_pos(2) <<" "<< target_pos(0) << " " << target_pos(1) <<" " << target_pos(2) << endl;
          trajs_writer << recent_traj.getPieceNum() <<" ";
          for( int i = 0 ; i < recent_traj.getPieceNum() ; i++)
          {
              Piece piece = recent_traj[i];
              Eigen::Matrix<double, 3, 6> cofmat = piece.getCoeffMat();
              trajs_writer << piece.getDuration() <<" ";
              for( int a = 0 ; a < 3 ; a++ )
              {
                for(int b = 0 ; b < 6 ; b++)
                {
                  trajs_writer << cofmat(a,b) <<" ";
                }
              }
          }
          trajs_writer << endl;
        }
      }
      else 
      {
        std::cout<<" [benchmark100] 前端失败，重新执行。 "<<std::endl;
        times++;
        continue;
      }


    }

#endif

#ifdef BENCHMARKTIME

    // coords_writer.open(TIMES_FILE);    
    coords_writer.open(CURVE_FILE);
    // trajs_writer.open(COORDS_FILE);

    srand(time(0));
    int times = 2000;
    int total_times = 0;
    int x_size  = floor(10*( pcsmap_manager -> boundary_xyzmax(0) - pcsmap_manager -> boundary_xyzmin(0) ));
    int y_size  = floor(10*( pcsmap_manager -> boundary_xyzmax(1) - pcsmap_manager -> boundary_xyzmin(1) ));
    int z_size  = floor(10*( pcsmap_manager -> boundary_xyzmax(2) - pcsmap_manager -> boundary_xyzmin(2) ));
    int x_begin = 10 * ceil(pcsmap_manager -> boundary_xyzmin(0));
    int y_begin = 10 * ceil(pcsmap_manager -> boundary_xyzmin(1));
    int z_begin = 10 * ceil(pcsmap_manager -> boundary_xyzmin(2));
    while( times-- ){
      
      total_times++;
      if(total_times > 10000){
        std::cout<<" [benchmark time] 重试次数达到最大值，终止。 "<<std::endl;
        break;
      }
      std::cout<<" [benchmark time] 执行第 "<< 2001 - times <<" /2000次 benchmark "<<std::endl;
      begin_pos(0) = 0.1 * ( rand() % x_size + x_begin );
      begin_pos(1) = 0.1 * ( rand() % y_size + y_begin );
      begin_pos(2) = 0.1 * ( rand() % z_size + z_begin );


      begin_vel(0) = 0;
      begin_vel(1) = 0;
      begin_vel(2) = 0;

      target_pos(0) = 0.1 * ( rand() % x_size + x_begin );
      target_pos(1) = 0.1 * ( rand() % y_size + y_begin );
      target_pos(2) = 0.1 * ( rand() % z_size + z_begin );

      target_vel(0) = 0;
      target_vel(1) = 0;
      target_vel(2) = 0;

      
      pcsmap_manager -> dropPoint(begin_pos);
      pcsmap_manager -> dropPoint(target_pos);

      if( (begin_pos - target_pos).norm() < 5 ){ 
        std::cout<<" [benchmark time] 起点距离终点太近，重新执行。 "<<std::endl;
        times++; continue; 
      }

      vector<Vector3d> pts;
      pts.push_back( begin_pos );
      pts.push_back( target_pos );

      renderPoints(pts, Vector3d(1,1,0) , 2.0, 1);

      // trajs_writer.precision(15); 
      // trajs_writer << begin_pos(0) << " " << begin_pos(1) <<" " << begin_pos(2) <<" "<< target_pos(0) << " " << target_pos(1) <<" " << target_pos(2) << endl;

      ros::Time before_planning = ros::Time::now();

      if( generatePath( begin_pos, target_pos ) )
      {

        generateTraj(recent_path, begin_vel, target_vel, ret_value);
        if( ret_value < 0){
          std::cout<<" [benchmark time] 优化失败，重新执行。 "<<std::endl;
          times++;
          continue;
        }
        else{
          ros::Time after_planning = ros::Time::now();
          double time_ms = 1000*(after_planning.toSec() - before_planning.toSec());
          double length;
          renderPath(recent_path);
          length = renderTraj(recent_traj);
          std::cout<<" [benchmark time] 成功，写入文件。 "<<std::endl;
          cout<<"长度="<<length<<endl;
          coords_writer.precision(15);
          // coords_writer <<length<<" "<<time_ms*0.3<< endl;

          //计算平均曲率
          double t_duration = recent_traj.getTotalDuration();
          int point_count = 0;
          Eigen::Vector3d pos, pos_pre;
          double d_theta, distance;
          double final_c = 0;
          for(double t = 0; t < t_duration - 0.05; t += 0.05)
          {
              pos     = recent_traj.getPos(t);
              pos_pre = recent_traj.getPos(t+0.05);
              d_theta = abs( atan2(pos_pre(1), pos_pre(0)) - atan2(pos(1), pos(0)));
              distance = (pos_pre - pos).head(2).norm();
              final_c += d_theta / distance;
              point_count++;
          }
          final_c /= point_count;
          coords_writer <<final_c<<endl;
        }
      }
      else 
      {
        std::cout<<" [benchmark time] 前端失败，重新执行。 "<<std::endl;
        times++;
        continue;
      }


    }
#endif


}

void PlannerManager::target2RcvCallBack(const geometry_msgs::PoseStamped& msg)
{
    // Vector3d begin_pos;
    // Vector3d target_pos;
    // target_pos(0) = msg.pose.position.x;
    // target_pos(1) = msg.pose.position.y;
    // target_pos(2) = msg.pose.position.z;

    // //cout<<"tr_cost = " << (pcsmap_manager->travelcost_map) -> getSDFValue(target_pos)<<endl;
    // //cout<<"tr_grad = " << (pcsmap_manager->travelcost_map) -> getSDFGrad(target_pos)<<endl;
    // double sdf_cost = (pcsmap_manager->occupancy_map) -> getSDFValue(target_pos);
    // cout<<"sdf_cost = " << sdf_cost <<endl;
    // cout<<"sdf_grad = " << (pcsmap_manager->occupancy_map) -> getSDFGrad(target_pos)<<endl;
    // //Vector3d up_normal = Vector3d(0,0,1);
    // //cout<<"plane_cost = " << 1 - up_normal.transpose() * (pcsmap_manager->travelcost_map) -> getTravelPN(target_pos)<<endl;

    // pcsmap_manager -> dropPoint(target_pos);
    // cout<<"target = " <<target_pos<<endl;

    // if(has_odom)
    // {
    //   begin_pos(0) = recent_odom.pose.pose.position.x;
    //   begin_pos(1) = recent_odom.pose.pose.position.y;
    //   //begin_pos(2) = recent_odom.pose.pose.position.z;
    //   begin_pos(2) = recent_odom.twist.twist.angular.z + 0.5;
    //   pcsmap_manager -> dropPoint(begin_pos);

    //   if( generatePath( begin_pos, target_pos ) )
    //   {
    //     generateTraj(recent_path);
    //   }
    // }

}


bool PlannerManager::generatePath( Vector3d start, Vector3d end )
{
  vector<Vector3d> path;
  astar_searcher -> AstarPathSearch( start , end );
  if( astar_searcher -> success_flag )
  {
    path = astar_searcher -> getPath();
    renderPath(path);
  }
  else
  {
    ROS_WARN("[A*] search failed.");
  }
  astar_searcher -> reset();
  recent_path = path;
  return (astar_searcher -> success_flag);
}


void PlannerManager::generateTraj( vector<Vector3d> path , Vector3d start_vel, Vector3d target_vel, int& ret_value)
{
  int N;
  int path_size = path.size();
  double temp_traj_parlength = traj_parlength;
  int index_gap = ceil( temp_traj_parlength / ((pcsmap_manager->occupancy_map) -> grid_resolution) );

  while( index_gap >= path_size - 1 ){
    temp_traj_parlength /= 1.5;
    index_gap = ceil( temp_traj_parlength / ((pcsmap_manager->occupancy_map) ->  grid_resolution) );
  }

  cout<<"H_range = "<<"["<<pcsmap_manager->min_height<<" -> "<<pcsmap_manager->max_height<<"]"<<endl;
  bool ret_opt;
  MatrixXd initState  = MatrixXd::Zero(3,3);
  MatrixXd finalState = MatrixXd::Zero(3,3);
  initState.col(0)  = path.front();
  initState.col(1)  = start_vel;
  finalState.col(0) = path.back();
  finalState.col(1) = target_vel;
  double gapi = (pcsmap_manager->occupancy_map) -> getHeightGap( path.front() );
  double gapf = (pcsmap_manager->occupancy_map) -> getHeightGap( path.back()  );
  double resolution = pcsmap_manager->occupancy_map->grid_resolution;
  if(gapi > pcsmap_manager->max_height) {gapi = pcsmap_manager->max_height;}
  if(gapf > pcsmap_manager->max_height) {gapf = pcsmap_manager->max_height;}
  if(pcsmap_manager->downproj){
    initState(2,0)  += resolution*2;
    finalState(2,0) += resolution*2;
  }
  else{
    initState(2,0)  += gapi - resolution*2;
    finalState(2,0) += gapf - resolution*2;
  }
  vector<Vector3d> Q;
  Vector3d wp;
  for( int ind = index_gap ; ind < path_size - 1 ; ind += index_gap ) 
  {
    wp = path[ind];
    //TODO 0.5 -> upLaser
  
    double gap = (pcsmap_manager->occupancy_map) -> getHeightGap( path[ind] );
    // wp(2) += 0.5;
    if(gap > pcsmap_manager->max_height) {gap = pcsmap_manager->max_height;}
    if(pcsmap_manager->downproj){
      wp(2) += resolution*2;
    }
    else{
      wp(2) += gap - resolution*2;
    }
    Q.push_back( wp );
  }
  renderPoints(Q, Vector3d(0.5,0.5,1.0), 0.2, 1);
  N = Q.size() + 1;
  ret_opt = minco_traj_optimizer -> generate_traj( initState, finalState, Q, N, recent_traj, false, ret_value);
  if(ret_opt == true)
  {
    renderTraj(recent_traj);
    publishTraj();
  }

}

void PlannerManager::publishTraj()
{
    mpc::Polynome poly;
    MatrixXd poses = recent_traj.getPositions();
    VectorXd ts    = recent_traj.getDurations();

    for (int i = 0; i < poses.cols(); i++)
    {
      geometry_msgs::Point temp;
      temp.x = poses(0, i);
      temp.y = poses(1, i);
      temp.z = poses(2, i);
      poly.pos_pts.push_back(temp);
    }
    for (int i = 0; i < ts.size(); i++)
    {
      poly.t_pts.push_back(ts(i));
    }
    poly.init_v.x = 0;
    poly.init_v.y = 0;
    poly.init_v.z = 0;
    poly.init_a.x = 0;
    poly.init_a.y = 0;
    poly.init_a.z = 0;
    poly.start_time = ros::Time::now();

    traj_pub.publish(poly);
}


void PlannerManager::renderPath( vector<Vector3d> path )
{
  visualization_msgs::Marker sphere, line_strip;
  sphere.header.frame_id  = line_strip.header.frame_id  = "world";
  sphere.header.stamp     = line_strip.header.stamp     = ros::Time::now();

  sphere.type             = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type         = visualization_msgs::Marker::LINE_STRIP;

  sphere.action           = line_strip.action           = visualization_msgs::Marker::ADD;
  sphere.id               = 0;
  line_strip.id           = 1;

  sphere.pose.orientation.w   = line_strip.pose.orientation.w  = 1.0;
  sphere.color.r              = line_strip.color.r             = 0.4;
  sphere.color.g              = line_strip.color.g             = 1.0;
  sphere.color.b              = line_strip.color.b             = 0.4;
  sphere.color.a              = line_strip.color.a             = 0.8;
  sphere.scale.x              = line_strip.scale.x             = 0.1;
  sphere.scale.y              = line_strip.scale.y             = 0.1;
  sphere.scale.z              = line_strip.scale.z             = 0.1;

  geometry_msgs::Point pt;
  Vector3d ptv;
  for(int i = 0 ; i < path.size(); i++)
  {
      ptv = path[i];
      pt.x = ptv(0);
      pt.y = ptv(1);
      pt.z = ptv(2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
  }
  path_vis_pub.publish(sphere);
  path_vis_pub.publish(line_strip);
}


double PlannerManager::renderTraj( Trajectory traj)
{
  double len = 0.0;
  int id = 4;
  visualization_msgs::Marker traj_vis, traj_des_vis;
  traj_vis.header.stamp       = ros::Time::now();
  traj_vis.header.frame_id    = "world";
  traj_vis.id   = id++;
  traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
  traj_vis.scale.x = 0.2;
  traj_vis.scale.y = 0.2;
  traj_vis.scale.z = 0.2;
  traj_vis.pose.orientation.x = 0.0;
  traj_vis.pose.orientation.y = 0.0;
  traj_vis.pose.orientation.z = 0.0;
  traj_vis.pose.orientation.w = 1.0;

  traj_vis.color.a = 1.0;
  traj_vis.color.r = 1.0;
  traj_vis.color.g = 1.0;
  traj_vis.color.b = 0.5;
  geometry_msgs::Point pt, pt_last,  pt_des, pt_des_last;
  Eigen::Vector3d pos;

  double t_duration = traj.getTotalDuration();
  vector<double> xs;
  vector<double> ys;
  vector<double> zs;
  double last_zs;
  for(double t = 0; t < t_duration - 0.05; t += 0.05)
  {
      pos = traj.getPos(t);
      xs.push_back(pos(0));
      ys.push_back(pos(1));
      len += (traj.getPos(t+0.05) - pos).head(2).norm();
      pos(2) += 0.0;
      if(pcsmap_manager->downproj)
      {
        pcsmap_manager -> deepDropPointToPC(pos);
      }
      // if( pos(2) < 5){
      //   pos(2) = last_zs;
      // }
      // else{ last_zs = pos(2);}
      zs.push_back(pos(2));

      // pt.x = pos(0);
      // pt.y = pos(1);
      // // pt.z = 0.0;
      // pt.z = pos(2);
      // traj_vis.points.push_back(pt);
  }

  traj_des_vis = traj_vis;
  traj_des_vis.id = traj_vis.id + 100000;

  double z = 0, z_des = 0;
  int list = 2;
  if(pcsmap_manager->downproj){
    list = 40;
  }
  for(int i = 0 ; i < zs.size() - list ; i++)
  {
    for(int k = 0 ; k < list - 1 ; k++){
      z = (z*k + zs[i+k])/(k+1);
    }
    pt.x = xs[i];
    pt.y = ys[i];
    pt.z = z;
    if(!pcsmap_manager->downproj){
      pt.z -= 0.25;
    }

    pt_des.x = xs[i];
    pt_des.y = ys[i];
    Vector3i ind = pcsmap_manager->occupancy_map->getGridIndex( Vector3d(pt.x, pt.y, pt.z) );
    Vector3d gz;
    pt_des.z = pcsmap_manager->zcost_map->getDistWithGradTrilinear(Vector3d(pt.x, pt.y, pt.z), gz);
    // if(pt_des.z < 0){cout<<pt_des.z<<endl;}
    // traj_vis.points.push_back(pt);
    if( traj_vis.points.size() == 0)
    {
      traj_vis.points.push_back(pt);
      continue;
    }
    if( traj_des_vis.points.size() == 0)
    {
      traj_des_vis.points.push_back(pt_des);
      continue;
    }

    double new_g = 2 * ( z / pcsmap_manager -> boundary_xyzmax(2));
    pt_last = traj_vis.points.back();
    pt_des_last = traj_des_vis.points.back();
    traj_vis.points.clear();
    traj_vis.color.g = (new_g > 1.0) ? 1.0 : new_g;
    traj_vis.id   = id++;
    traj_vis.points.push_back(pt_last);
    traj_vis.points.push_back(pt);
    traj_vis_pub.publish(traj_vis);

// if(!pcsmap_manager->downproj)
// {
//     traj_des_vis.points.clear();
//     traj_des_vis.id   = id++;
//     traj_des_vis.color.r = 0;
//     traj_des_vis.color.g = 0;
//     traj_des_vis.color.b = 1;
//     traj_des_vis.points.push_back(pt_des_last);
//     traj_des_vis.points.push_back(pt_des);
//     traj_vis_pub.publish(traj_des_vis);
// }

#ifdef NORMAL
    // ros::Duration(0.001).sleep();
#endif

  }

  // traj_vis_pub.publish(traj_vis);
  return len;
}

void PlannerManager::renderPoints(vector<Eigen::Vector3d> pts, Eigen::Vector3d color , double scale, int id)
{
    visualization_msgs::Marker sphere;
    sphere.header.frame_id  = "world";
    sphere.header.stamp     = ros::Time::now();

    sphere.type             = visualization_msgs::Marker::SPHERE_LIST;
    sphere.action           = visualization_msgs::Marker::ADD;
    sphere.id               = id;
    sphere.pose.orientation.w   = 1.0;
    sphere.color.r              = color(0);
    sphere.color.g              = color(1);
    sphere.color.b              = color(2);
    sphere.color.a              = 0.8;
    sphere.scale.x              = scale;
    sphere.scale.y              = scale;
    sphere.scale.z              = scale;
    geometry_msgs::Point pt;
    Eigen::Vector3d ptv;
    for(int i = 0 ; i < pts.size(); i++)
    {
        ptv = pts[i];
        pt.x = ptv(0);
        pt.y = ptv(1);
        pt.z = ptv(2);
        sphere.points.push_back(pt);
    }
    point_vis_pub.publish(sphere);
}