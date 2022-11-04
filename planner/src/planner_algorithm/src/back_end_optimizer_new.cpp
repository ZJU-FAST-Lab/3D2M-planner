#include <planner_algorithm/back_end_optimizer.h>
#include <planner_algorithm/lbfgs.hpp>


void TrajOpt::addTimeIntPenalty(double& cost) {
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
  for (int i = 0; i < N; ++i) {
    const auto& c = jerkOpt.b.block<6, 3>(i * 6, 0);
    step = jerkOpt.T1(i) / K;
    s1 = 0.0;
    innerLoop = K + 1;

    for (int j = 0; j < innerLoop; ++j) {
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      alpha = 1.0 / K * j;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;

      omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

    
      if (grad_cost_p(pos, grad_tmp, cost_tmp)) {
        
        gradViolaPc = beta0 * grad_tmp.transpose();
        gradViolaPt = alpha * grad_tmp.dot(vel);
        (this->jerkOpt).gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
        (this->jerkOpt).gdT(i) += omg * (cost_tmp / K + step * gradViolaPt);
        cost += omg * step * cost_tmp;
      }
    
      

      if (grad_cost_v(vel, grad_tmp, cost_tmp)) {
        gradViolaVc = beta1 * grad_tmp.transpose();
        gradViolaVt = alpha * grad_tmp.dot(acc);
        (this->jerkOpt).gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
        (this->jerkOpt).gdT(i) += omg * (cost_tmp / K + step * gradViolaVt);
        cost += omg * step * cost_tmp;
      }
      if (grad_cost_a(acc, grad_tmp, cost_tmp)) {
        gradViolaAc = beta2 * grad_tmp.transpose();
        gradViolaAt = alpha * grad_tmp.dot(jer);
        (this->jerkOpt).gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
        (this->jerkOpt).gdT(i) += omg * (cost_tmp / K + step * gradViolaAt);
        cost += omg * step * cost_tmp;
      }

      if (grad_cost_shape(pos,vel, grad_tmp_p, cost_tmp_p,grad_tmp_v, cost_tmp_v)) {        
        gradViolaPc = beta0 * grad_tmp_p.transpose()+beta1 * grad_tmp_v.transpose();
        gradViolaPt = alpha * grad_tmp_p.dot(vel)+alpha * grad_tmp_v.dot(acc);
        (this->jerkOpt).gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
        (this->jerkOpt).gdT(i) += omg * (cost_tmp_p / K + step * gradViolaPt);
        cost += omg * step * cost_tmp_p;

      }   

      s1 += step;
    }
  }
}


static double rhoP_tmp_;

// SECTION  variables transformation and gradient transmission
static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}

static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}

static inline double gdT2t(double t) {
  if (t > 0) {
    return t + 1.0;
  } else {
    double denSqrt = (0.5 * t - 1.0) * t + 1.0;
    return (1.0 - t) / (denSqrt * denSqrt);
  }
}

static void forwardT(const double& t, Eigen::Ref<Eigen::VectorXd> vecT) {
  vecT.setConstant(expC2(t));
}

static void addLayerTGrad(const double& t,
                          const Eigen::Ref<const Eigen::VectorXd>& gradT,
                          double& gradt) {
  gradt = gradT.sum() * gdT2t(t);
}

// !SECTION variables transformation and gradient transmission

// SECTION object function
static inline double objectiveFunc(void* ptrObj,
                                   const Eigen::VectorXd &x,
                                   Eigen::VectorXd &grad) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  //const double& t = x[0];
  //double& gradt = grad[0];
  VectorXd VT(obj.N);
  //Eigen::Map<const Eigen::MatrixXd> T( x , 1, (obj.dim_t) );
  //Eigen::Map<const Eigen::MatrixXd> P(x + (obj.dim_t), 3, (obj.dim_p) );
  //Eigen::Map<Eigen::MatrixXd> gradT(grad , 1, (obj.dim_t) );
  //Eigen::Map<Eigen::MatrixXd> gradP(grad + (obj.dim_t), 3, (obj.dim_p) );

  //VT = T.row(0);
  double t = x(0);
  //VT = VectorXd::Ones( (obj.N) ) * T(0,0);
  forwardT(t, VT);

  MatrixXd P = MatrixXd::Zero(3,obj.dim_p);
  MatrixXd GP;
  for(int i = 0 ; i < obj.dim_p ; i++)
  {
    P(0,i) = x(i*3 + 1);
    P(1,i) = x(i*3 + 2);
    P(2,i) = x(i*3 + 3);
  }

  (obj.jerkOpt).generate(P, (obj.finalS), VT);
  double cost = (obj.jerkOpt).getTrajJerkCost();

  if( std::isnan(cost) )
  {
    cout<< "1 nan t = "<<t <<std::endl;
  }

  (obj.jerkOpt).calGrads_CT();
  obj.addTimeIntPenalty(cost);



  (obj.jerkOpt).calGrads_PT();
  (obj.jerkOpt).gdT.array() += (obj.rhoT);

  cost += (obj.rhoT) * VT.sum();


  addLayerTGrad(t , (obj.jerkOpt).gdT, grad(0));
  //gradT = (obj.jerkOpt).gdT.transpose();
  GP = (obj.jerkOpt).gdP;

  for (int i = 0; i < (obj.dim_p); ++i) 
  {
    //P.col(i) = Q[i];
    grad(i*3 +1)   = GP(0,i);
    grad(i*3 +2)   = GP(1,i);
    grad(i*3 +3)   = GP(2,i);

  }

  if( std::isnan(cost) )
  {
    cout<< "nan "<<std::endl;
  }

  return cost;
}


// !SECTION object function
static inline int earlyExit(void* ptrObj,
                            const Eigen::VectorXd &x,
                            const Eigen::VectorXd &grad,
                            const double fx,
                            const double step,
                            int k,
                            int ls) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  std::cout<<"t="<<x(0)<<std::endl;
  //if (obj.pause_debug_) {
  // if (false) {
    
  //   VectorXd VT(obj.N);
  //   Eigen::Map<const Eigen::MatrixXd> T( x , 1, (obj.dim_t) );
  //   Eigen::Map<const Eigen::MatrixXd> P( x + (obj.dim_t) , 3, (obj.dim_p) );

  //   //VT = VectorXd::Ones( (obj.N) ) * T(0,0);
    
  //   forwardT(T(0,0), VT);
  //   obj.jerkOpt.generate(P, obj.finalS, VT);
  //   auto traj = obj.jerkOpt.getTraj();
  //   obj.drawDebug(traj, P);


  //   // NOTE pause
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }
  return k > 3*1e3;
}

bool TrajOpt::generate_traj(const Eigen::MatrixXd& initState,
                            const Eigen::MatrixXd& finalState,
                            const std::vector<Eigen::Vector3d>& Q,
                            const int N,
                            Trajectory& traj,
                            bool keep_result) {

  this->N = N;
  
  
  //this->dim_t = N;
  this->dim_t = 1;
  this->dim_p = N - 1;

  this->x = VectorXd::Zero((this->dim_t) + (this->dim_p)*3);

  VectorXd VT(N);
  //Eigen::Map<Eigen::MatrixXd> T( (this->x) , 1, (this->dim_t) );
  //Eigen::Map<Eigen::MatrixXd> P( (this->x) + (this->dim_t) , 3, (this->dim_p) );
  MatrixXd P = MatrixXd::Zero(3,(this->dim_p));

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
  this->x(0) = logC2(T0);
  //t = logC2(T0);
 
  for (int i = 0; i < N - 1; ++i) {
    //P.col(i) = Q[i];
    this -> x(i*3 +1)   = Q[i](0);
    this -> x(i*3 +2)   = Q[i](1);
    this -> x(i*3 +3)   = Q[i](2);

  }
  (this->jerkOpt).reset(initState, N);
  // NOTE optimization
  lbfgs::lbfgs_parameter_t lbfgs_params;
  //lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 128;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 0;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = 1e-7;
  //lbfgs_params.line_search_type = 0;
  double minObjectiveXY , minObjectiveZ;

  rhoP_tmp_ = (this->rhoP);


  // auto opt_ret1 = lbfgs::lbfgs_optimize((this->dim_t) + 3 * (this->dim_p), 
  //                                      this->x, 
  //                                      &minObjectiveXY,
  //                                      &objectiveFunc, nullptr,
  //                                      &earlyExit, this, &lbfgs_params);

  std::cout<<"t0  ="<<this->x(0)<<std::endl;
  auto opt_ret1 = lbfgs::lbfgs_optimize(this->x, 
                                       minObjectiveXY,
                                       &objectiveFunc, nullptr,
                                       &earlyExit, this, lbfgs_params);

  std::cout << "\033[32m"
            << "ret: " << opt_ret1 << "\033[0m" << std::endl;
  if (this->pause_debug) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  if (opt_ret1 < 0) {
    return false;
  }

  forwardT( this->x(0) , VT);
  //VT = VectorXd::Ones(N) * T(0,0);
  for(int i = 0 ; i < this->dim_p ; i++)
  {
    P(0,i) = this->x(i*3 + 1);
    P(1,i) = this->x(i*3 + 2);
    P(2,i) = this->x(i*3 + 3);
  }
  (this->jerkOpt).generate(P, finalState, VT);
  traj = (this->jerkOpt).getTraj();

  return true;
}

bool TrajOpt::grad_cost_p(const Eigen::Vector3d& p,
                          Eigen::Vector3d& gradp,
                          double& costp) {
  
  Vector3d p_drop = p;
  p_drop(2) -= 0.0;
  double sdf_value = 0 ;
  double truncation = 0.5;
  double travel_cost = 0;
  double exp_travel_cost = 0;

  Eigen::Vector3d gp(Eigen::Vector3d::Zero());
  Eigen::Vector3d gtr(Eigen::Vector3d::Zero());

  costp = 0;
  gradp = Eigen::Vector3d::Zero();

  //重力
  //costp     += 10 * p(2);
  //gradp(2)  += 10;

  //任务：修复梯度
  //sdf_value = (environment->occupancy_map) -> getSDFValue(p);
  //gp        = (environment->occupancy_map) -> getSDFGrad(p);

  sdf_value = (environment->occupancy_map) -> getDistWithGradTrilinear(p, gp);

  // travel cost
  exp_travel_cost = (environment->travelcost_map) -> getSDFValue(p_drop);
  travel_cost = log(exp_travel_cost + 1);
  gtr         = -(environment->travelcost_map) -> getSDFGrad(p_drop);
  gtr         = gtr / (exp_travel_cost + 1);
  gtr(2)      = 0;


  if ( sdf_value <=  truncation && sdf_value != 0 )
  {    
    costp +=  rhoP_tmp_ * pow (truncation - sdf_value, 3);
    gradp +=  rhoP_tmp_ * 3 * pow(truncation - sdf_value, 2) * (-gp);
    //costp +=  rhoP_tmp_ * sdf_value;
    //gradp +=  rhoP_tmp_ * gp;

  }
  // if( travel_cost > 0)
  // {
  //   costp +=  0.4 * rhoP_tmp_ * pow( travel_cost,3 );
  //   gradp +=  0.4 * rhoP_tmp_ * 3 * pow( travel_cost,2 ) * gtr;

  // }   
  return true;
  
}

bool TrajOpt::grad_cost_v(const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradv,
                          double& costv) {
  
  double vpen  = v.squaredNorm() - (this->vmax) * (this->vmax);
  if (vpen > 0) {
    gradv = this->rhoV * 6 * vpen * vpen * v;
    costv = this->rhoV * vpen * vpen * vpen;
    return true;
  }
  return false;
}

bool TrajOpt::grad_cost_a(const Eigen::Vector3d& a,
                          Eigen::Vector3d& grada,
                          double& costa) {

  grada = Eigen::Vector3d::Zero();
  costa = 0;
  double apen  = a.squaredNorm() - (this->amax) * (this->amax);

  if (apen > 0) {
    grada += (this->rhoA) * 6 * apen * apen * a;
    costa += (this->rhoA) * apen * apen * apen;
    return true;
  }
 
  return false;
}


bool TrajOpt::grad_cost_shape(const Eigen::Vector3d& p,const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradp,
                          double& costp,Eigen::Vector3d& gradv,double& costv) {

  double footprint_length = 0.4;
  double footprint_width  = 0.5; 
  costp = 0;
  gradp = Eigen::Vector3d::Zero();
  costv = 0;
  gradv = Eigen::Vector3d::Zero();
  //哎 不能考虑法线导致的roll和picth，因为它会随着环境而改变，不能写成关于xyz及其导数的形式，没法传递？
  //反正现在改水平了
  Eigen::Vector3d n_p = p;
  // environment->dropPoint(n_p);
  Eigen::Vector3d n_y = v;n_y(2)=0;//n_y=n_y.normalized();
  Eigen::Vector3d n_z(0,0,1);
  Eigen::Vector3d n_x = n_y.cross(n_z);//n_y.normalized().cross(n_z.normalized());

  vector<Eigen::Vector3d> pts;
  double travel_cost=0;
  Eigen::Vector3d gtr_p(0,0,0),gtr_for_v_3dim_tmp(0,0,0),gtr_v(0,0,0);
  Eigen::Vector2d gtr_for_v_2dim_tmp(0,0);
  Eigen::Matrix2d pass1_numberator,pass2_numberator,pass_numberator;
  Eigen::Matrix2d pass1_denominator,pass2_denominator,pass_denominator;

  for(double lamda1=-1;lamda1<=1;lamda1+=1)
  {
    for(double lamda2=-1;lamda2<=1;lamda2+=1)
    {

      Eigen::Vector3d P = n_p + lamda1 * footprint_width*0.5*n_y.normalized()+lamda2*footprint_length*0.5*n_x.normalized();
      travel_cost += (environment->travelcost_map)   -> getSDFValue(P);
      gtr_p       += (-(environment->travelcost_map) -> getSDFGrad(P));

      if(lamda2==0 && lamda1==0)
      {
        continue;
      }

      gtr_for_v_3dim_tmp=-(environment->travelcost_map)-> getSDFGrad(P);

      if(gtr_for_v_3dim_tmp.norm()==0)
      {
        continue;
      }
      //算传导的矩阵pass
      double k1=lamda1*footprint_width*0.5;
      double k2=lamda2*footprint_length*0.5;
      Eigen::Matrix2d I;I << 1,0,0,1;//tmd我单位阵写了个1111
      Eigen::Matrix2d B;B<<0,-1,1,0;  //B'Y轴=X轴  分子构型 复合起来 (分子形式的)*B'  分母构型  B*(分母形式的)  但这里这个()又他妈的是个对陈阵
      Eigen::Vector2d V(n_y(0),n_y(1));
      Eigen::Vector2d V_X(n_y(1),-n_y(0));
      double t=V.norm();
      Eigen::Vector2d t0=B.transpose()*V;

      // // std::cout<<"-------"<<std::endl;
      // pass1_numberator=(I/p0_derivate_2_norm-p0_derivate_multi_its_trans/pow(p0_derivate_2_norm,3));//对陈阵  汪博论文里用的这个
      // // std::cout<<"汪博论文 y轴那个矩阵 分子构型 "<<pass1_numberator<<std::endl;
      // pass1_numberator=(I/t)-1/pow(t,3)*V*V.transpose();
      // // std::cout<<"网站 y轴那个矩阵 分子构型 "<<pass1_numberator<<std::endl;
      // pass2_numberator=(I/x_axis_2_norm-x_axis_multi_its_trans/pow(x_axis_2_norm,3))*B.transpose();//(分子形式的)*B'
      // // std::cout<<"x轴_分子构型_我的复合版本 "<<pass2_numberator<<std::endl;
      // pass2_numberator=((1/((V.transpose()*B).norm()))*B).transpose()-1/pow(t0.norm(),3)*t0*(V.transpose()*B*B.transpose());
      // // std::cout<<"网站 x轴 分子构型"<<pass2_numberator<<std::endl;
      // pass_numberator=k1*pass1_numberator+k2*pass2_numberator;
      // // std::cout<<"pass_numberator 分子 "<<pass_numberator<<std::endl;

      // gtr_for_v_2dim_tmp(0)=gtr_for_v_3dim_tmp(0);
      // gtr_for_v_2dim_tmp(1)=gtr_for_v_3dim_tmp(1);
      // gtr_for_v_2dim_tmp   =gtr_for_v_2dim_tmp.transpose()*pass_numberator;//分子这么求
      // // std::cout<<"分子构型 "<<gtr_for_v_2dim_tmp<<std::endl;

      //好家伙 这是个对称阵
      pass1_denominator=(I/pow(t,1))-1/pow(t,3)*V*V.transpose();
      // std::cout<<"我算的y轴那个矩阵 分母构型"<<pass1_denominator<<std::endl;
      // pass1_denominator=(I*(V.norm())-(V*(((I.transpose()*V)).transpose()))/V.norm())/pow(V.norm(),2);
      // // std::cout<<"hzc y轴那个矩阵 分母构型"<<pass1_denominator<<std::endl;
      // pass2_denominator=(B*(V.norm())-(V*(((B.transpose()*V)).transpose()))/V.norm())/pow(V.norm(),2);
      // // std::cout<<"hzc x轴那个矩阵 分母构型"<<pass2_denominator<<std::endl;
      pass2_denominator=B*((I/pow(t,1))-1/pow(t,3)*V_X*V_X.transpose());//B*(分母形式的)
      // std::cout<<"x轴_分子构型_我的复合版本 "<<pass2_denominator<<std::endl;

      pass_denominator=k1*pass1_denominator+k2*pass2_denominator;
      // std::cout<<"pass_denominator 分母 "<<pass_denominator<<std::endl;
      
      gtr_for_v_2dim_tmp(0)=gtr_for_v_3dim_tmp(0);
      gtr_for_v_2dim_tmp(1)=gtr_for_v_3dim_tmp(1);
      gtr_for_v_2dim_tmp   =pass_denominator*gtr_for_v_2dim_tmp;//分母要这么求
      // std::cout<<"分母构型 "<<gtr_for_v_2dim_tmp<<std::endl;

      gtr_for_v_3dim_tmp(0)=gtr_for_v_2dim_tmp(0);
      gtr_for_v_3dim_tmp(1)=gtr_for_v_2dim_tmp(1);
      gtr_for_v_3dim_tmp(2)=0;
  // }

      gtr_v+=gtr_for_v_3dim_tmp;
    }
  }

  travel_cost=travel_cost/9;//这几个要除一起除
  gtr_p=gtr_p/9;
  gtr_v=gtr_v/9;
  //这里感觉除不除以9算是系数问题不影响是否-1004？
  Eigen::Vector3d gtr_vis=-gtr_p;
  if(gtr_p.norm()!=0)
  {
    // vis_grad(n_p,gtr_vis,n_p(0)*100+n_p(1)*10+n_p(2));
  }


  if( travel_cost > 0)
  {
    costp +=  rhoP_tmp_ * pow( travel_cost,3 );
    // costv =this->rhoV* pow( travel_cost,3 );
    // costv =0;
    gradp +=  rhoP_tmp_ * 3 * pow( travel_cost,2 ) * gtr_p;
    gradv +=  rhoP_tmp_ * 3 * pow( travel_cost,2 ) * gtr_v;
    // std::cout<<"gradv "<<gradv<<std::endl;
  }
  return true;
}



void TrajOpt::drawDebug(Trajectory end_path , Eigen::Map<const Eigen::MatrixXd> P)
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

void TrajOpt::drawDebugWp(std::vector<Eigen::Vector3d> front_path)
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
