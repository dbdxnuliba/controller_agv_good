/*************************************************************************
  > File Name: model_predictive_control.cpp
  > Author: TAI Lei
  > Mail: ltai@ust.hk
  > Created Time: Wed Apr 17 11:48:46 2019
 ************************************************************************/

#include "model_predictive_control.h"

namespace bz_robot {
MPC::MPC()
{
  last_result_.resize(6*SIZE_PREDICTION-2,0.0);
  first_slove_ = true;
}

MPC::~MPC()
{

}

void MPC::UpdateState(const float &a, float &delta, State &state)
{
  if (delta >= default_param_.max_steer_angle) delta = default_param_.max_steer_angle;
  if (delta <= - default_param_.max_steer_angle) delta = - default_param_.max_steer_angle;

  state.x = state.x + state.v * std::cos(state.yaw) * DT;
  state.y = state.y + state.v * std::sin(state.yaw) * DT;
  state.yaw = state.yaw + state.v / default_param_.wheel_distance * CppAD::tan(delta) * DT;
  state.v = state.v + a * DT;

  if (state.v > default_param_.max_speed) state.v = default_param_.max_speed;
  if (state.v < default_param_.min_speed) state.v = default_param_.min_speed;
}

void MPC::CalcRefTrajectory(Vec_f &var, Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f sp, float dl, M_XREF &xref)
{
  xref = M_XREF::Zero();
  int ncourse = cx.size();
  State si(0.0,0.0,0.0,0.0);
  //用于可视化
  Eigen::Vector3f foot_point;
  std::vector<Eigen::Vector3f> foot_points;
  foot_points.clear();

  for(int i=0; i< SIZE_PREDICTION; i++)
  {
    //更新i时刻预测状态
    si.x = var[default_param_.x_start+i];
    si.y = var[default_param_.y_start+i];
    si.yaw = var[default_param_.yaw_start+i];
    si.v = var[default_param_.v_start+i];
    //寻找i时刻的最近点
    int ind = track_.CalculateNearestIndex(si);
    //std::cout<<"ind="<<ind<<std::endl;
    if (ind<ncourse)
    {
      xref(0, i) = cx[ind];
      xref(1, i) = cy[ind];
      xref(2, i) = cyaw[ind];
      xref(3, i) = sp[ind];
    }
    else
    {
      xref(0, i) = cx[ncourse - 1];
      xref(1, i) = cy[ncourse - 1];
      xref(2, i) = cyaw[ncourse - 1];
      xref(3, i) = sp[ncourse - 1];
    }
    foot_point[0] = xref(0, i);
    foot_point[1] = xref(1, i);
    foot_point[2] = xref(2, i);

    foot_points.push_back(foot_point);
  }
  //std::cout<<"机器人偏向角："<<var[default_param_.yaw_start]<<" 线路偏向角："<<xref(2, 0)<<std::endl;
  bz_robot::MessageToRos::GetInstance()->VisualFootPoint(foot_points);
}
#if 0
Vec_f MPC::MpcSolve(State x0)
{
  if(track_.IsGoReached(x0))
  {
    Vec_f v1;
    SetZero(v1);
    last_result_.resize(6*SIZE_PREDICTION-2,0.0);
    first_slove_ = true;
    return v1;
  }
  //1.定义求解过程中需要的变量
  auto t1 = std::chrono::high_resolution_clock::now();
  if(x0.v < default_param_.initial_velocity)
  {
    x0.v = default_param_.initial_velocity;
  }
  size_t n_vars = SIZE_PREDICTION * 4 + (SIZE_PREDICTION - 1) * 2;//变量个数，包括SIZE_PREDICTION个时刻状态量和SIZE_PREDICTION-1个时刻的控制量
  size_t n_constraints = SIZE_PREDICTION * 4 + (SIZE_PREDICTION - 2) * 2;  //约束个数
  SplineParam track_param = track_.GetTrack();
  //2.初始化输入
  //std::cout<<"*******************************"<<std::endl;
  Dvector vars(n_vars);
  UpdateInitialInput(x0);
  WrapInput(track_param);
  for (int i = 0; i < n_vars; i++)
  {
    //vars[i] = 0.0;
    vars[i] = last_result_[i];
    VisualPredictionPoint(last_result_);
  }

  // 3.确定输入量的上下边界
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  SetVarBound(vars,vars_lowerbound,vars_upperbound);
  //4.设置约束的上下边界
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  SetConstraintBound(vars,constraints_lowerbound,constraints_upperbound);
  //5.计算预测轨迹
  M_XREF traj_ref;
  CalcRefTrajectory(last_result_,track_param.x,track_param.y,track_param.yaw,track_param.allowed_speed,0.02,traj_ref);
  FG_EVAL fg_eval(traj_ref);
  //5.1根据轨迹的垂足点重新更新变量边界约束
  //SetRoadBound(traj_ref,vars,vars_lowerbound,vars_upperbound);
  //6.设置求解器的参数
  std::string options;
  SetSloverOption(options);

  // 7.进行求解
  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_EVAL>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
  //bool ok = true;
  //ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  //std::cout<<"solution.status ="<<solution.status<<std::endl;
  Vec_f result;
  result.resize(6*SIZE_PREDICTION-2,0.0);
  //todo 求解成功用结果对变量赋值，求解失败就用上一次的求解结果进行处理后赋值
  if(solution.status != CppAD::ipopt::solve_result<Dvector>::success)
  {
    PrintfSolutionStatus(solution.status);
  }
  if(solution.status == CppAD::ipopt::solve_result<Dvector>::maxiter_exceeded)
  {
    //处理控制量,最后一个时刻控制量保持不变
    for(int i=0;i<SIZE_PREDICTION-1;i++)
    {
      result[default_param_.a_start+i]   = 0.0;
      result[default_param_.delta_start+i]   = 0.0;
    }
    //处理状态量,最后一个时刻按照最终控制量进行更新
    float beta = std::atan(0.5*tan(last_result_[default_param_.delta_start]));
    for(int i=0;i<SIZE_PREDICTION-1;i++)
    {
      result[default_param_.x_start+i]   = x0.x + 0.1 * std::cos(x0.yaw+beta)*DT*i;
      result[default_param_.y_start+i]   = x0.y + 0.1 * std::sin(x0.yaw+beta)*DT*i;
      result[default_param_.yaw_start+i] = x0.yaw ;
      result[default_param_.v_start+i]   = 0.1;
    }
    result[default_param_.x_start+SIZE_PREDICTION-1]   = x0.x + 0.1 * std::cos(x0.yaw+beta)*DT*(SIZE_PREDICTION-1);
    result[default_param_.y_start+SIZE_PREDICTION-1]   = x0.y + 0.1 * std::sin(x0.yaw+beta)*DT*(SIZE_PREDICTION-1);
    result[default_param_.yaw_start+SIZE_PREDICTION-1] = x0.yaw ;
    result[default_param_.v_start+SIZE_PREDICTION-1]   = 0.1;
  }
  else
  {
    result.clear();
    for (auto i =0 ; i < n_vars; i++)
    {
      result.push_back((float)solution.x[i]);
    }
  }

  MixPredictiveAndAllowed(result,x0);
  //當偏差太大，且有繼續擴大的趨勢
  float delta_unwap = 0.0;
  if(NeedNewInput(x0,track_param,delta_unwap))
  {
    //result.at(default_param_.v_start+1) = std::min(result.at(default_param_.v_start+1),(float)0.5*default_param_.max_speed);
    result.at(default_param_.delta_start)= delta_unwap;
  }
  //auto t2 = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> delta_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  //std::cout<<"优化耗时为:"<<delta_time.count()<<std::endl;
  return result;
}
#endif
Vec_f MPC::MpcSolve(const Pose<FLOAT_T> &state, const ControlData &current_vel, const Msg<Pose<FLOAT_T> > &goal)
{

  //1.如果判断机器人已经超过目标位置，则需要返回零值
  ControlData control = current_vel;
  State current_state(0.0,0.0,0.0,0.0);
  //转化到机器人中心
  control.velocity = FromBase(control.velocity,control.steer_angle);
  current_state.x   = state.position.x;
  current_state.y   = state.position.y;
  current_state.yaw = state.heading_angle;
  current_state.v   = control.velocity;
  if(track_.IsGoReached(current_state))
  {
    Vec_f v1;
    SetZero(v1);
    last_result_.resize(6*SIZE_PREDICTION-2,0.0);
    first_slove_ = true;
    return v1;
  }
  //1.定义求解过程中需要的变量
  auto t1 = std::chrono::high_resolution_clock::now();
  if(current_state.v < default_param_.initial_velocity)
  {
    current_state.v = default_param_.initial_velocity;
  }
  size_t n_vars = SIZE_PREDICTION * 4 + (SIZE_PREDICTION - 1) * 2;//变量个数，包括SIZE_PREDICTION个时刻状态量和SIZE_PREDICTION-1个时刻的控制量
  size_t n_constraints = SIZE_PREDICTION * 4 + (SIZE_PREDICTION - 2) * 2;  //约束个数
  SplineParam track_param = track_.GetTrack();
  //2.初始化输入
  //std::cout<<"*******************************"<<std::endl;
  Dvector vars(n_vars);
  UpdateInitialInput(current_state);
  WrapInput(track_param);
  for (int i = 0; i < n_vars; i++)
  {
    //vars[i] = 0.0;
    vars[i] = last_result_[i];
    VisualPredictionPoint(last_result_);
  }

  // 3.确定输入量的上下边界
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  SetVarBound(vars,vars_lowerbound,vars_upperbound);
  //4.设置约束的上下边界
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  SetConstraintBound(vars,constraints_lowerbound,constraints_upperbound);
  //5.计算预测轨迹
  M_XREF traj_ref;
  CalcRefTrajectory(last_result_,track_param.x,track_param.y,track_param.yaw,track_param.allowed_speed,0.02,traj_ref);
  FG_EVAL fg_eval(traj_ref);
  //5.1根据轨迹的垂足点重新更新变量边界约束
  //SetRoadBound(traj_ref,vars,vars_lowerbound,vars_upperbound);
  //6.设置求解器的参数
  std::string options;
  SetSloverOption(options);

  // 7.进行求解
  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_EVAL>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
  //bool ok = true;
  //ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  //std::cout<<"solution.status ="<<solution.status<<std::endl;
  Vec_f result;
  result.resize(6*SIZE_PREDICTION-2,0.0);
  //todo 求解成功用结果对变量赋值，求解失败就用上一次的求解结果进行处理后赋值
  if(solution.status != CppAD::ipopt::solve_result<Dvector>::success)
  {
    PrintfSolutionStatus(solution.status);
  }
  if(solution.status == CppAD::ipopt::solve_result<Dvector>::maxiter_exceeded)
  {
    //处理控制量,最后一个时刻控制量保持不变
    for(int i=0;i<SIZE_PREDICTION-1;i++)
    {
      result[default_param_.a_start+i]   = 0.0;
      result[default_param_.delta_start+i]   = 0.0;
    }
    //处理状态量,最后一个时刻按照最终控制量进行更新
    float beta = std::atan(0.5*tan(last_result_[default_param_.delta_start]));
    for(int i=0;i<SIZE_PREDICTION-1;i++)
    {
      result[default_param_.x_start+i]   = current_state.x + 0.1 * std::cos(current_state.yaw+beta)*DT*i;
      result[default_param_.y_start+i]   = current_state.y + 0.1 * std::sin(current_state.yaw+beta)*DT*i;
      result[default_param_.yaw_start+i] = current_state.yaw ;
      result[default_param_.v_start+i]   = 0.1;
    }
    result[default_param_.x_start+SIZE_PREDICTION-1]   = current_state.x + 0.1 * std::cos(current_state.yaw+beta)*DT*(SIZE_PREDICTION-1);
    result[default_param_.y_start+SIZE_PREDICTION-1]   = current_state.y + 0.1 * std::sin(current_state.yaw+beta)*DT*(SIZE_PREDICTION-1);
    result[default_param_.yaw_start+SIZE_PREDICTION-1] = current_state.yaw ;
    result[default_param_.v_start+SIZE_PREDICTION-1]   = 0.1;
  }
  else
  {
    result.clear();
    for (auto i =0 ; i < n_vars; i++)
    {
      result.push_back((float)solution.x[i]);
    }
  }

  MixPredictiveAndAllowed(result,current_state);
  //當偏差太大，且有繼續擴大的趨勢
  float delta_unwap = 0.0;
  if(NeedNewInput(current_state,track_param,delta_unwap))
  {
    //result.at(default_param_.v_start+1) = std::min(result.at(default_param_.v_start+1),(float)0.5*default_param_.max_speed);
    result.at(default_param_.delta_start)= delta_unwap;
  }
  //auto t2 = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> delta_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  //std::cout<<"优化耗时为:"<<delta_time.count()<<std::endl;
  return result;
}

void MPC::SetTrack(const PathData &path, const ControlData &data)
{
  //转化到机器人中心
  float velocity = FromBase(data.velocity,data.steer_angle);
  track_.DealWithPath(path,velocity);
}
//当偏差太大，且车头偏向角在往偏差大的一边偏转，重新初始化
void MPC::NewInitialInput(const State &x0)
{
  std::cout<<"橫向偏差大於0.4m,且有繼續擴大的趨勢"<<std::endl;
  //0时刻，更新为实际值
  last_result_[default_param_.x_start]   = x0.x;
  last_result_[default_param_.y_start]   = x0.y;
  last_result_[default_param_.yaw_start] = x0.yaw;
  last_result_[default_param_.v_start]   = default_param_.initial_velocity;

  for(int i=1;i<SIZE_PREDICTION;i++)
  {
    last_result_[default_param_.x_start+i]   = last_result_[default_param_.x_start+i-1] + default_param_.initial_velocity*DT*cos(x0.yaw);
    last_result_[default_param_.y_start+i]   = last_result_[default_param_.x_start+i-1] + default_param_.initial_velocity*DT*sin(x0.yaw);
    last_result_[default_param_.yaw_start+i] = x0.yaw;
    last_result_[default_param_.v_start+i]   = default_param_.initial_velocity;
  }
  //size-1时刻,保持不变
}

void MPC::UpdateInitialInput(const State &x0)
{
  //开机第一次进行求解计算，对last_result_进行初始化
  if(first_slove_)
  {
    //std::cout<<"初始化last_result_"<<std::endl;
    for(int i=0;i<SIZE_PREDICTION;i++)
    {
      last_result_[default_param_.x_start+i]   = x0.x + x0.v*i*DT*std::cos(x0.yaw);
      last_result_[default_param_.y_start+i]   = x0.y + x0.v*i*DT*std::sin(x0.yaw);
      last_result_[default_param_.yaw_start+i] = x0.yaw;
      last_result_[default_param_.v_start+i]   = x0.v;
    }
    for(int i=0;i<SIZE_PREDICTION-1;i++)
    {
      last_result_[default_param_.delta_start+i]   = 0.0;
      last_result_[default_param_.a_start+i]   = 0.0;
    }
    //first_slove_ = false;
  }
  else
  {
    for(int i=1;i<SIZE_PREDICTION;i++)
    {
      last_result_[default_param_.x_start+i-1]   = last_result_[default_param_.x_start+i];
      last_result_[default_param_.y_start+i-1]   = last_result_[default_param_.y_start+i];
      last_result_[default_param_.yaw_start+i-1] = last_result_[default_param_.yaw_start+i];
      last_result_[default_param_.v_start+i-1]   = last_result_[default_param_.v_start+i];
    }
    //size时刻,状态量保持不变
    //0时刻，状态量更新为实际值
    last_result_[default_param_.x_start]   = x0.x;
    last_result_[default_param_.y_start]   = x0.y;
    last_result_[default_param_.yaw_start] = x0.yaw;
    last_result_[default_param_.v_start]   = x0.v;

    for(int i=1;i<SIZE_PREDICTION-1;i++)
    {
      last_result_[default_param_.delta_start+i-1]   = last_result_[default_param_.delta_start+i];
      last_result_[default_param_.a_start+i-1]   = last_result_[default_param_.a_start+i];
    }
    //size时刻,控制量为0
    last_result_[default_param_.delta_start+SIZE_PREDICTION-2] = 0.0;
    last_result_[default_param_.a_start+SIZE_PREDICTION-2] = 0.0;
  }
}

void MPC::SetZero(Vec_f &v)
{
  v.clear();
  v.resize(6*SIZE_PREDICTION-2,0.0);
}

void MPC::MixPredictiveAndAllowed(Vec_f &result, State &state)
{
  //1.对机器人速度进行限制
  for(int i=0;i<SIZE_PREDICTION;i++)
  {
    State si(result[default_param_.x_start+i],result[default_param_.y_start+i],0.0,0.0);
    int index = track_.CalculateNearestIndex(si);
    float v_max = track_.GetTrack().allowed_speed.at(index);
    clamp(result[default_param_.v_start + i],default_param_.initial_velocity,v_max);
  }
  //4.更新last state and input          ^~
  last_result_.clear();
  last_result_.insert(last_result_.end(),result.begin(),result.end());
//  for (auto i = default_param_.delta_start; i < default_param_.delta_start+SIZE_PREDICTION-1; i++)
//  {
//    std::cout<<"delta="<<last_result_[i]<<std::endl;
//  }
}

void MPC::WrapInput(const SplineParam &track_param)
{
  //0.儲存終點座標
  Eigen::Vector3f end_point(track_param.x.back(),track_param.y.back(),track_param.yaw.back());
  //1.如果尾部点很接近终点，说明已经处理过，也不需要处理
  float dx = std::fabs(last_result_.at(default_param_.x_start+SIZE_PREDICTION-1) - end_point[0]);
  float dy = std::fabs(last_result_.at(default_param_.y_start+SIZE_PREDICTION-1) - end_point[1]);
  if(dx<0.02 && dy<0.02)
  {
    return;
  }
  //2.寻找超出终点的点，从尾部点开始遍历，当尾部点超出终点，继续往前遍历
  int index = 65535;//第一个超出目标点的点号
  State si(0.0,0.0,0.0,0.0);
  for(int i=SIZE_PREDICTION-1;i>=0;i--)
  {
    si.x = last_result_.at(default_param_.x_start+i);
    si.y = last_result_.at(default_param_.y_start+i);
    int nearest_point_id = track_.CalculateNearestIndex(si);
    if(nearest_point_id == track_param.x.size()-1)
    {
      index = i;
      //std::cout<<"需要对last_result_进行处理"<<std::endl;
    }
    //当尾部点也没有超出终点，不需要改变，直接返回；
    if(index == 65535)
    {
      return;
    }
  }
  //处理x,y,yaw,v，終點處理要保證能儘量計算出較大的控制量，進行偏差調整,因此
  for(int i=index;i<SIZE_PREDICTION;i++)
  {
    last_result_.at(default_param_.x_start+i)   = end_point[0]+0.01*cos(end_point[2]);
    last_result_.at(default_param_.y_start+i)   = end_point[1]+0.01*sin(end_point[2]);
    last_result_.at(default_param_.yaw_start+i) = last_result_.at(default_param_.yaw_start+index);
    last_result_.at(default_param_.v_start+i)   = 0.0;
  }
  //处理a,delta
  for(int i=index;i<SIZE_PREDICTION-1;i++)
  {
    last_result_.at(default_param_.a_start+i) = 0.0;
    last_result_.at(default_param_.delta_start+i) = 0.0;
  }
}

void MPC::SetVarBound(const Dvector &var, Dvector &lower_bound, Dvector &upper_bound)
{
  //std::cout<<"所有时刻中的最大速度为："<<max_speed<<std::endl;
  // 先将值初始化为一个很大的值,保证不会出现无法求解的情况,然后对需要专门限制的变量进行限制
  for (auto i = 0; i < var.size(); i++)
  {
    lower_bound[i] = -1000.0;
    upper_bound[i] = 1000.0;
  }
  //x的范围
  for (auto i = default_param_.x_start; i < default_param_.x_start+SIZE_PREDICTION; i++)
  {
//    lower_bound[i] = var[i]-default_param_.max_speed*DT;
//    upper_bound[i] = var[i]+default_param_.max_speed*DT;
    lower_bound[i] = var[default_param_.x_start]-default_param_.max_speed*DT*i;
    upper_bound[i] = var[default_param_.x_start]+default_param_.max_speed*DT*i;
  }
  //y的范围
  for (auto i = default_param_.y_start; i < default_param_.y_start+SIZE_PREDICTION; i++)
  {
//    lower_bound[i] = var[i]-default_param_.max_speed*DT;
//    upper_bound[i] = var[i]+default_param_.max_speed*DT;
    lower_bound[i] = var[default_param_.y_start]-default_param_.max_speed*DT*(i-default_param_.y_start);
    upper_bound[i] = var[default_param_.y_start]+default_param_.max_speed*DT*(i-default_param_.y_start);
  }
  //yaw的范围
  for (auto i = default_param_.yaw_start; i < default_param_.yaw_start+SIZE_PREDICTION; i++)
  {
    lower_bound[i] = -3.15;
    upper_bound[i] = 3.15;
  }
  //v的取值范围
  for (auto i = default_param_.v_start; i < default_param_.v_start+SIZE_PREDICTION; i++)
  {
    lower_bound[i] = 0.3;
    upper_bound[i] = default_param_.max_speed;
  }
  //a的取值范围
  for (auto i = default_param_.a_start; i < default_param_.a_start+SIZE_PREDICTION-1; i++)
  {
    lower_bound[i] = -default_param_.max_accel;
    upper_bound[i] = default_param_.max_accel;
  }
  //delta的取值范围
  for (auto i = default_param_.delta_start; i < default_param_.delta_start+SIZE_PREDICTION-1; i++)
  {
    if(first_slove_)
    {
      lower_bound[i] = -default_param_.max_steer_angle;
      upper_bound[i] = default_param_.max_steer_angle;
      first_slove_ = false;
    }
    else
    {
      lower_bound[i] = -default_param_.max_steer_angle;
      upper_bound[i] = default_param_.max_steer_angle;
      //todo  t时刻车头偏向角忽然从上一次计算t时刻的0弧度，到下一次计算的0.35弧度，约束只能限制用一次计算的各个时刻
      lower_bound[i] = last_result_[i]-default_param_.max_steer_angle*DT;
      if(lower_bound[i]<-default_param_.max_steer_angle)
      {
        lower_bound[i] = -default_param_.max_steer_angle;
      }
      if(lower_bound[i]>default_param_.max_steer_angle)
      {
        lower_bound[i] = default_param_.max_steer_angle;
      }
      upper_bound[i] = last_result_[i]+default_param_.max_steer_angle*DT;
      if(upper_bound[i]<-default_param_.max_steer_angle)
      {
        upper_bound[i] = -default_param_.max_steer_angle;
      }
      if(upper_bound[i]>default_param_.max_steer_angle)
      {
        upper_bound[i] = default_param_.max_steer_angle;
      }

    }
  }

//  for (auto i = default_param_.delta_start; i < default_param_.delta_start+SIZE_PREDICTION-1; i++)
//  {
//    lower_bound[i] = -default_param_.max_steer_angle;
//    upper_bound[i] = default_param_.max_steer_angle;
//  }
//  lower_bound[default_param_.delta_start] = last_result_[default_param_.delta_start]-default_param_.max_steer_angle*DT;
//  upper_bound[default_param_.delta_start] = last_result_[default_param_.delta_start]+default_param_.max_steer_angle*DT;
}

void MPC::SetConstraintBound(const Dvector &var, Dvector &lower_bound, Dvector &upper_bound)
{
  //因为是等式,所以上下约束相等
  for (auto i = 0; i < lower_bound.size(); i++)
  {
    lower_bound[i] = 0.0;
    upper_bound[i] = 0.0;
  }
  lower_bound[default_param_.x_start]   = var[default_param_.x_start];
  lower_bound[default_param_.y_start]   = var[default_param_.y_start];
  lower_bound[default_param_.yaw_start] = var[default_param_.yaw_start];
  lower_bound[default_param_.v_start]   = var[default_param_.v_start];

  upper_bound[default_param_.x_start]   = var[default_param_.x_start];
  upper_bound[default_param_.y_start]   = var[default_param_.y_start];
  upper_bound[default_param_.yaw_start] = var[default_param_.yaw_start];
  upper_bound[default_param_.v_start]   = var[default_param_.v_start];
  //控制量变化率的约束
  for(int i=0; i<SIZE_PREDICTION-2; i++)
  {
    lower_bound[default_param_.a_start+i]     = -default_param_.max_accel*DT;
    upper_bound[default_param_.a_start+i]     = default_param_.max_accel*DT;
    lower_bound[default_param_.delta_start+i-1]   = -default_param_.max_steer_angle*DT;
    upper_bound[default_param_.delta_start+i-1]   = default_param_.max_steer_angle*DT;
  }
}

void MPC::SetSloverOption(std::string &option)
{
  // Uncomment this if you'd like more print information
  option += "Integer print_level  0\n";
  //注意：将sparse设置为true可使求解器利用稀疏例程，这会使计算更加快速。
  //如果您可以取消注释其中之一，并查看是否有所不同，但是如果您都取消注释，
  //则计算时间应增加几个数量级。
  //options += "Sparse  true        forward\n";
  option += "Sparse  true        reverse\n";
  option += "Integer max_iter      50\n";
  option += "Numeric tol          1e-5\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
  option += "Numeric max_cpu_time          0.3\n";
}

bool MPC::NeedNewInput(const State &x, const SplineParam &track, float &delta)
{
  int index = track_.CalculateNearestIndex(x);
  //最近點爲終點，不需要處理，直接返回false
  if(index == track.x.size()-1)
  {
    return false;
  }
  State x_current(track.x.at(index),track.y.at(index),track.yaw.at(index),0.0);
  State x_next(track.x.at(index+1),track.y.at(index+1),track.yaw.at(index+1),0.0);
  //判斷機器人位於線路的左側還是右側，左正右負
  float path_angle = atan2(x_next.y-x_current.y,x_next.x-x_current.x);
  float angle1     = atan2(x.y-x_current.y,x.x-x_current.x);
  float angle      = angles::normalize_angle(angle1-path_angle);
  float dx         = x.x-x_current.x;
  float dy         = x.y-x_current.y;
  float dis        = std::sqrt(dx*dx+dy*dy);
  //機器人位於線路右側時
  if(angle<0)
  {
    dis *= -1.0;
  }
  float angle_diff = angles::normalize_angle(x.yaw-path_angle);
  //車頭偏向是否會使橫向偏差繼續擴大,如果不是擴大則返回false
  if(dis*angle_diff<=0.0)
  {
    return false;
  }
  else
  {
    //偏离轨道量较小，不需要重新初始化
    if(std::fabs(dis)<0.3)
    {
      return false;
    }
    //偏离轨道量较大
    else
    {
      delta = - dis/std::fabs(dis)*default_param_.max_steer_angle*x.v/(default_param_.max_speed*0.5);
      return true;
    }
  }
}

void MPC::VisualPredictionPoint(const Vec_f &var)
{
  Eigen::Vector3f point;
  std::vector<Eigen::Vector3f> predict_point;
  for(int i=0;i<SIZE_PREDICTION;i++)
  {
    point[0] = var[default_param_.x_start+i];
    point[1] = var[default_param_.y_start+i];
    point[2] = var[default_param_.yaw_start+i];

    predict_point.push_back(point);
  }
  bz_robot::MessageToRos::GetInstance()->VisualMpcPredictPoint(predict_point);
}

void MPC::SetRoadBound(const M_XREF &trajectory_point, const Dvector vars, Dvector &lower_bound, Dvector &upper_bound)
{
//  //x的范围
//  for (auto i = default_param_.x_start; i < default_param_.x_start+SIZE_PREDICTION; i++)
//  {
//    lower_bound[i] = var[default_param_.x_start]-default_param_.max_speed*DT*(i-default_param_.x_start);
//    upper_bound[i] = var[default_param_.x_start]+default_param_.max_speed*DT*(i-default_param_.x_start);
//  }
//  //y的范围
//  for (auto i = default_param_.y_start; i < default_param_.y_start+SIZE_PREDICTION; i++)
//  {
//    lower_bound[i] = var[default_param_.y_start]-default_param_.max_speed*DT*(i-default_param_.y_start);
//    upper_bound[i] = var[default_param_.y_start]+default_param_.max_speed*DT*(i-default_param_.y_start);
//  }
//  //yaw的范围
//  for (auto i = default_param_.yaw_start; i < default_param_.yaw_start+SIZE_PREDICTION; i++)
//  {
//    lower_bound[i] = -3.15;
//    upper_bound[i] = 3.15;
//  }
  //v的取值范围
  for (auto i = default_param_.v_start; i < default_param_.v_start+SIZE_PREDICTION; i++)
  {
    lower_bound[i] = trajectory_point(3,i-default_param_.v_start);
    upper_bound[i] = trajectory_point(3,i-default_param_.v_start);
  }
}

void MPC::PrintfSolutionStatus(const int &status)
{
  switch(status)
  {
    case 0:
      std::cout<<"求解状态未定义（not_defined），求解器没有针对当前问题返回求解结果"<<std::endl;
      break;
    case 1:
      std::cout<<"求解成功（success），求解结果达到了要求的收敛误差"<<std::endl;
      break;
    case 2:
      std::cout<<"求解达到最大迭代次数（maxiter_exceeded），超过了最大迭代次数"<<std::endl;
      break;
    case 3:
      std::cout<<"求解下降速度太慢（stop_at_tiny_step），算法终止，因为收敛速度非常慢"<<std::endl;
      break;
    case 4:
      std::cout<<"求解收敛在一个可接受的精度，但是未达到最佳的精度（stop_at_acceptable_point），"
                 "算法终止，因为进度非常慢。"<<std::endl;
      break;
    case 5:
      std::cout<<"求解结果不可靠（local_infeasibility），算法收敛到一个不可行的点（问题可能没有解决方案）"
                 ""<<std::endl;
      break;
    case 6:
      std::cout<<"求解返回值不能发生（user_requested_stop），此返回值不应发生"<<std::endl;
      break;
    case 7:
      std::cout<<"求解找到了可行点（feasible_point_found）"<<std::endl;
      break;
    case 8:
      std::cout<<"求解迭代发散（diverging_iterates）"<<std::endl;
      break;
    case 9:
      std::cout<<"求解恢复失败（restoration_failure），恢复阶段失败，算法不知道如何进行"<<std::endl;
      break;
    case 10:
      std::cout<<"求解计算搜索方向失败（error_in_step_computation），Ipopt尝试计算搜索方向"
                 "时发生了不可恢复的错误"<<std::endl;
      break;
    case 11:
      std::cout<<"求解遇到无效数字（invalid_number_detected），如inf或nan等"<<std::endl;
      break;
    case 12:
      std::cout<<"求解自由度太小（too_few_degrees_of_freedom），有太多的相等约束，"
                 "或者修复了太多的变量（IPOPT删除了固定变量），则会发生这种情况。"<<std::endl;
      break;
    case 13:
      std::cout<<"求解遇到未知的内部错误（internal_error）"<<std::endl;
      break;
    case 14:
      std::cout<<"求解返回未定义的状态（unknown）"<<std::endl;
      break;
    default:
      break;
  }

}

}

