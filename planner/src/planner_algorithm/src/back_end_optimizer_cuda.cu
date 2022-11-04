#include <planner_algorithm/back_end_optimizer.h>
#include <planner_algorithm/lbfgs.hpp>

#include <cuda_runtime.h>

__device__ bool grad_cost_p(void* ptrObj,
                          const Eigen::Vector3d& p,
                          Eigen::Vector3d& gradp,
                          double& costp) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  double sdf_value = 0 ;
  double truncation = 1.0;
  double travel_cost = 0;
  Eigen::Vector3d gp(Eigen::Vector3d::Zero());
  Eigen::Vector3d gtr(Eigen::Vector3d::Zero());
  costp = 0;
  gradp = Eigen::Vector3d::Zero();
  sdf_value = (obj.environment->occupancy_map) -> getDistWithGradTrilinear(p, gp);
  // travel cost
  travel_cost = (obj.environment->travelcost_map) -> getSDFValue(p);
  gtr         = -(obj.environment->travelcost_map) -> getSDFGrad(p);

  if ( sdf_value <=  truncation && sdf_value != 0 )
  {    
    costp +=  obj.rhoP * pow (truncation - sdf_value, 3);
    gradp +=  obj.rhoP * 3 * pow(truncation - sdf_value, 2) * (-gp);
  }
  if( travel_cost > 0)
  {
    costp +=  obj.rhoP * pow( travel_cost,3 );
    gradp +=  obj.rhoP * 3 * pow( travel_cost,2 ) * gtr;
  }
  return true;
}

__device__ bool grad_cost_v(void* ptrObj,
                          const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradv,
                          double& costv) {
  
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  double vpen  = v.squaredNorm() - (obj.vmax) * (obj.vmax);
  if (vpen > 0) {
    gradv = obj.rhoV * 6 * vpen * vpen * v;
    costv = obj.rhoV * vpen * vpen * vpen;
    return true;
  }
  return false;
}

__device__ bool grad_cost_a(void* ptrObj,
                          const Eigen::Vector3d& a,
                          Eigen::Vector3d& grada,
                          double& costa) {

  TrajOpt& obj = *(TrajOpt*)ptrObj;
  grada = Eigen::Vector3d::Zero();
  costa = 0;
  double apen  = a.squaredNorm() - (obj.amax) * (obj.amax);

  if (apen > 0) {
    grada += (obj.rhoA) * 6 * apen * apen * a;
    costa += (obj.rhoA) * apen * apen * apen;
    return true;
  }
 
  return false;
}

__global__ void addCost(void* ptrObj, double* cost, int n)
{
    TrajOpt& obj = *(TrajOpt*)ptrObj;
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d grad_tmp, grad_tmp_p, grad_tmp_v ;
    double cost_tmp, cost_tmp_p, cost_tmp_v;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg;
    int innerLoop;

    // 获取全局索引
    int i = blockIdx.x;
    int j = threadIdx.x;
    ///////////////////////
    innerLoop = obj.K + 1;
    const auto& c = obj.jerkOpt.b.block<6, 3>(i * 6, 0);
    s1 = 0.0;
    step = obj.jerkOpt.T1(i) / obj.K;
    for(int l = 0; l < i; l++){ s1 += obj.jerkOpt.T1(l); }
    s1 += step * j;

    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    beta0 << 1.0, s1, s2, s3, s4, s5;
    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
    alpha = 1.0 / obj.K * j;
    pos = c.transpose() * beta0;
    vel = c.transpose() * beta1;
    acc = c.transpose() * beta2;
    jer = c.transpose() * beta3;

    omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

    if (grad_cost_p(ptrObj,pos, grad_tmp, cost_tmp)) {
        
      gradViolaPc = beta0 * grad_tmp.transpose();
      gradViolaPt = alpha * grad_tmp.dot(vel);
      obj.jerkOpt.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
      obj.jerkOpt.gdT(i) += omg * (cost_tmp / obj.K + step * gradViolaPt);
      (*cost) += omg * step * cost_tmp;
    }
    if (grad_cost_v(ptrObj,vel, grad_tmp, cost_tmp)) {
      gradViolaVc = beta1 * grad_tmp.transpose();
      gradViolaVt = alpha * grad_tmp.dot(acc);
      obj.jerkOpt.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
      obj.jerkOpt.gdT(i) += omg * (cost_tmp / obj.K + step * gradViolaVt);
      (*cost) += omg * step * cost_tmp;
    }
    if (grad_cost_a(ptrObj,acc, grad_tmp, cost_tmp)) {
      gradViolaAc = beta2 * grad_tmp.transpose();
      gradViolaAt = alpha * grad_tmp.dot(jer);
      obj.jerkOpt.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
      obj.jerkOpt.gdT(i) += omg * (cost_tmp / obj.K + step * gradViolaAt);
      (*cost) += omg * step * cost_tmp;
    }
}


__global__ void TrajOpt::addTimeIntPenalty(double& cost) {

    int nBytes = 1 * sizeof(double);
    // 申请host内存
    double *cost_host;
      cost_host  = (double*)malloc(nBytes);
    (*cost_host) = 0;

    // 申请device内存
    double *cost_device;
    cudaMalloc((void**)&cost_device, nBytes);

    // 将host数据拷贝到device
    cudaMemcpy((void*)cost_device, (void*)cost_host, nBytes, cudaMemcpyHostToDevice);

    // 定义kernel的执行配置
    dim1 blockSize( (this->N) );
    dim1 gridSize(  (this->K) + 1 );
    // 执行kernel
    addCost << < gridSize, blockSize >> >( this, cost_device, this->N);

    // 将device得到的结果拷贝到host
    cudaMemcpy((void*)cost_host, (void*)cost_device, nBytes, cudaMemcpyDeviceToHost);

    // 检查执行结果
    cout << "cost: " << (*cost_host) << endl;
    cudaFree(cost_device);
    free(cost_host);

    cost = *(cost_host);
}




static double rhoP_tmp_;

// SECTION  variables transformation and gradient transmission
__host__ static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}

__host__ static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}

__host__ static inline double gdT2t(double t) {
  if (t > 0) {
    return t + 1.0;
  } else {
    double denSqrt = (0.5 * t - 1.0) * t + 1.0;
    return (1.0 - t) / (denSqrt * denSqrt);
  }
}

__host__ static void forwardT(const double& t, Eigen::Ref<Eigen::VectorXd> vecT) {
  vecT.setConstant(expC2(t));
}

__host__ static void addLayerTGrad(const double& t,
                          const Eigen::Ref<const Eigen::VectorXd>& gradT,
                          double& gradt) {
  gradt = gradT.sum() * gdT2t(t);
}

// !SECTION variables transformation and gradient transmission

// SECTION object function
__host__ static inline double objectiveFunc(void* ptrObj,
                                   const double* x,
                                   double* grad,
                                   const int n) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  //const double& t = x[0];
  //double& gradt = grad[0];
  VectorXd VT(obj.N);
  Eigen::Map<const Eigen::MatrixXd> T( x , 1, (obj.dim_t) );
  Eigen::Map<const Eigen::MatrixXd> P(x + (obj.dim_t), 3, (obj.dim_p) );
  Eigen::Map<Eigen::MatrixXd> gradT(grad , 1, (obj.dim_t) );
  Eigen::Map<Eigen::MatrixXd> gradP(grad + (obj.dim_t), 3, (obj.dim_p) );

  //VT = T.row(0);
  double t = T(0,0);
  //VT = VectorXd::Ones( (obj.N) ) * T(0,0);
  forwardT(t, VT);
  (obj.jerkOpt).generate(P, (obj.finalS), VT);

  double cost = (obj.jerkOpt).getTrajJerkCost();
  (obj.jerkOpt).calGrads_CT();
  obj.addTimeIntPenalty(cost);

  (obj.jerkOpt).calGrads_PT();
  (obj.jerkOpt).gdT.array() += (obj.rhoT);

  cost += (obj.rhoT) * VT.sum();

  addLayerTGrad(t , (obj.jerkOpt).gdT, gradT(0,0));
  //gradT = (obj.jerkOpt).gdT.transpose();
  gradP = (obj.jerkOpt).gdP;

  return cost;
}


// !SECTION object function
__host__ static inline int earlyExit(void* ptrObj,
                            const double* x,
                            const double* grad,
                            const double fx,
                            const double xnorm,
                            const double gnorm,
                            const double step,
                            int n,
                            int k,
                            int ls) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  //if (obj.pause_debug_) {
  if (false) {
    
    VectorXd VT(obj.N);
    Eigen::Map<const Eigen::MatrixXd> T( x , 1, (obj.dim_t) );
    Eigen::Map<const Eigen::MatrixXd> P( x + (obj.dim_t) , 3, (obj.dim_p) );

    //VT = VectorXd::Ones( (obj.N) ) * T(0,0);
    
    forwardT(T(0,0), VT);
    obj.jerkOpt.generate(P, obj.finalS, VT);
    auto traj = obj.jerkOpt.getTraj();
    obj.drawDebug(traj, P);


    // NOTE pause
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return k > 1e3;
}

__host__ bool TrajOpt::generate_traj(const Eigen::MatrixXd& initState,
                            const Eigen::MatrixXd& finalState,
                            const std::vector<Eigen::Vector3d>& Q,
                            const int N,
                            Trajectory& traj,
                            bool keep_result) {

  this->N = N;
  
  
  //this->dim_t = N;
  this->dim_t = 1;
  this->dim_p = N - 1;

  this->x = new double[ (this->dim_t) + 3 * (this->dim_p)];

  VectorXd VT(N);
  Eigen::Map<Eigen::MatrixXd> T( (this->x) , 1, (this->dim_t) );
  Eigen::Map<Eigen::MatrixXd> P( (this->x) + (this->dim_t) , 3, (this->dim_p) );

  // NOTE set boundary conditions
  (this->initS)  = initState;
  (this->finalS) = finalState;
  double tempNorm = (this->initS).col(1).norm(); // v0
  (this->initS).col(1) *= tempNorm > (this->vmax) ? ((this->vmax) / tempNorm) : 1.0;
  tempNorm = (this->initS).col(2).norm(); //a0
  (this->initS).col(2) *= tempNorm > (this->amax) ? ((this->amax) / tempNorm) : 1.0;

  // set initial guess
  /* 
  double len0 = (initState.col(0)  - Q[0]).norm();
  double lenf = (finalState.col(0) - Q[N-1]).norm();
  T(0,0)   = len0 / (this->vmax);
  T(0,N-1) = lenf / (this->vmax);
  for (int i = 1; i < N - 1; i++)
  {
    T(0,i) =  (Q[i]  - Q[i-1]).norm() / (this->vmax);
  }
  */
  double len = 0.0;
  len += (initState.col(0)  - Q[0]).norm(); 
  len += (finalState.col(0)  - Q[N-1]).norm();
  for (int i = 1; i < N - 1; i++)
  {
    len +=  (Q[i]  - Q[i-1]).norm() ;
  }
  double T0 = len / N / (this->vmax);
  //T(0,0) = T0;
  T(0,0) = logC2(T0);
  //t = logC2(T0);
 
  for (int i = 0; i < N - 1; ++i) {
    P.col(i) = Q[i];
  }
  (this->jerkOpt).reset(initState, N);
  // NOTE optimization
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 128;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 1e-32;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = 1e-5;
  lbfgs_params.line_search_type = 0;
  double minObjectiveXY , minObjectiveZ;

  rhoP_tmp_ = (this->rhoP);


  auto opt_ret1 = lbfgs::lbfgs_optimize((this->dim_t) + 3 * (this->dim_p), 
                                       this->x, 
                                       &minObjectiveXY,
                                       &objectiveFunc, nullptr,
                                       &earlyExit, this, &lbfgs_params);

  std::cout << "\033[32m"
            << "ret: " << opt_ret1 << "\033[0m" << std::endl;
  if (this->pause_debug) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  if (opt_ret1 < 0) {
    if( !keep_result )
    {
      delete[] this->x;
    }
    return false;
  }

  forwardT(T(0,0), VT);
  //VT = VectorXd::Ones(N) * T(0,0);
  (this->jerkOpt).generate(P, finalState, VT);
  traj = (this->jerkOpt).getTraj();
  if( !keep_result )
  {
    delete[] this->x;
  }
  
  return true;
}


__host__ void TrajOpt::drawDebug(Trajectory end_path , Eigen::Map<const Eigen::MatrixXd> P)
{
  int id = 0;
  visualization_msgs::Marker sphere, line_strip;
  sphere.header.frame_id = line_strip.header.frame_id = "world";
  sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
  sphere.id = id;
  line_strip.id = id + 1000;
  id++;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.r = 1;
  sphere.color.g = 0.5;
  sphere.color.b = 0.5;
  sphere.color.a = 1;
  line_strip.color.r = 1;
  line_strip.color.g = 0;
  line_strip.color.b = 1;
  line_strip.color.a = 1;
  sphere.scale.x = 0.1;
  sphere.scale.y = 0.1;
  sphere.scale.z = 0.1;
  line_strip.scale.x = 0.05 / 2;
  geometry_msgs::Point pt;

  double dur = end_path.getDurations().sum();
  for (double i = 0; i < dur - 1e-4; i+=0.1)
  {
    Eigen::Vector3d dur_p = end_path.getPos(i);
    pt.x = dur_p(0);
    pt.y = dur_p(1);
    pt.z = dur_p(2);
    line_strip.points.push_back(pt);
  }

  for (double i = 0; i < P.cols(); i++)
  {
    pt.x = P(0, i);
    pt.y = P(1, i);
    pt.z = P(2, i);
    sphere.points.push_back(pt);
  }

  debug_pub.publish(line_strip);
  debug_pub.publish(sphere);
}

__host__ void TrajOpt::drawDebugWp(std::vector<Eigen::Vector3d> front_path)
{
  int id = 0;
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id++;
  // kino_pub_.publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = 0;
  mk.color.g = 1;
  mk.color.b = 0;
  mk.color.a = 1;

  mk.scale.x = 0.075;
  mk.scale.y = 0.075;
  mk.scale.z = 0.075;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(front_path.size()); i++) {
    pt.x = front_path[i](0);
    pt.y = front_path[i](1);
    pt.z = 0;
    mk.points.push_back(pt);
  }
  debug_wp_pub.publish(mk);
  ros::Duration(0.001).sleep();
}
