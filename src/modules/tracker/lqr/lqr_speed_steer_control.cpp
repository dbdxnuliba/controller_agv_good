/*************************************************************************
  > File Name: lqr_speed_steer_control.cpp
  > Author: cao pan
  > Mail:
  > Created Time: 2020/09/03
 ************************************************************************/

#include "lqr_speed_steer_control.h"
#include <chrono>

namespace bz_robot {

LQR::LQR()
{
}

LQR::~LQR()
{

}

void LQR::SetTrack(const PathData &path, const ControlData &data)
{
  //转化到机器人中心
  float velocity = FromBase(data.velocity,data.steer_angle);
  track_.DealWithPath(path,velocity);
}

void LQR::ClearTrack()
{
  track_.ClearSpline();
}

std::vector<Pose<FLOAT_T> > LQR::GetTrackNew()
{
  std::vector<Pose<FLOAT_T> > paths;
  Pose<FLOAT_T> path(0.0,0.0,0.0);
  paths.clear();
  SplineParam spline = track_.GetTrack();
  for(int i=0;i<spline.x.size();i++)
  {
    path.position.x = spline.x.at(i);
    path.position.y = spline.y.at(i);
    paths.push_back(path);
  }
  return paths;
}

Matrix4f LQR::SolveDare(Matrix4f A, Vector4f B, Matrix4f Q, float R)
{
  Matrix4f X = Q;//立卡提矩阵
  int maxiter = 150;
  float eps = 0.01;
  Matrix4f AT = A.transpose();
  RowVector4f BT = B.transpose();
  for(int i=0; i<maxiter; i++)
  {
    //Matrix5f Xn = AT*X*A-AT*X*B*(R+BT*X*B).inverse() * BT*X*A+Q;
    Matrix4f Xn = AT*X*(A-B/(R+BT*X*B) * BT*X*A)+Q;
    Matrix4f error = Xn - X;
    //.cwiseAbs()元素取绝对值，.maxCoeff()取最大的元素
    if(error.cwiseAbs().maxCoeff()<eps)
    {
      return Xn;
    }
    X = Xn;
  }
  return X;
}

RowVector4f LQR::dlqr(Matrix4f A, Vector4f B, Matrix4f Q, float R)
{
  Matrix4f X = SolveDare(A, B ,Q, R);
  RowVector4f BT = B.transpose();
  RowVector4f K = 1.0/(BT*X*B + R) * (BT*X*A);
  return K;
}
//VectorX2<float> LQR::ComputeLqr(const State &state, const float &delta_in)
ControlData LQR::ComputeLqr(const Pose<FLOAT_T> &state, const ControlData &current_vel, const Msg<Pose<FLOAT_T> > &goal)
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
    control.velocity    = 0.0;
    control.steer_angle = 0.0;
    control.dt          = param_.dt;
    //ClearTrack();
    return control;
  }
  int ind = 0;
  SplineParam track_param = track_.GetTrack();
  Vec_f cx = track_param.x;
  Vec_f cy = track_param.y;
  Vec_f cyaw = track_param.yaw;
  Vec_f sp= track_param.allowed_speed;
  float e = ComputeLateralError(current_state,cx,cy,cyaw,ind);//机器人位于线路左侧为正

  float tv = sp[ind];
  //计算前瞻角，防止曲率变化对角度偏差的影响
  float lookahead_ratio = 5.0;
  lookahead_ratio = lookahead_ratio*current_vel.velocity;
  int point = ind + 10*lookahead_ratio;
  int point1 = ind + 30;
  int max_point = track_param.yaw.size()-1;
  clamp(point,0,max_point);
  clamp(point1,0,max_point);
  float th_e = angles::normalize_angle(current_state.yaw - cyaw[point]);
  float forward_delta = 0.0;
  if(point1 == max_point)
  {
    forward_delta = forward_delta - angles::normalize_angle(current_state.yaw - goal.data.heading_angle);//最后一段距离主要调整角度偏差
  }
  Matrix4f A = Matrix4f::Zero();
  A(0, 0) = 1.0;
  A(0 ,1) = DT;
  A(1 ,2) = current_state.v;
  A(2 ,2) = 1.0;
  A(2 ,3) = DT;

  Vector4f B = Vector4f::Zero();
  B(3) = current_state.v/L;

  //Q,R权重系数,todo 是否增加几道屏障，保证横向偏差大时快速调整，偏差小时放慢调整
  //判断依据为横向偏差和角度偏差，将横向偏差和角度偏差进行权重相加，当综合误差小于某个值时尽量减小偏转角度
  float error_weighed = (0.4*e + 0.6*th_e)/0.5;
  Matrix4f Q = Matrix4f::Identity();
  float R    = 1.0;
//  //综合偏差很小，并且横向偏差很小，只做小转角的调整
//  if(std::fabs(error_weighed)<=0.15 && std::fabs(e)<0.15)
//  {
//    Q(0,0)     = 0.6;//横向偏差1.5
//    Q(1,1)     = 0.5;//横向偏差变化率
//    Q(2,2)     = 0.8;//角度偏差
//    Q(3,3)     = 0.5;//角度偏差变化率
//    R          = 5.0;//车头偏向
//  }
//  //综合偏差很小，并且横向偏差很小，只做小转角的调整
//  else if(std::fabs(error_weighed)<=0.15 && std::fabs(e)>0.15)
//  {
//    Q(0,0)     = 8.5;//横向偏差1.5
//    Q(1,1)     = 1.0;//横向偏差变化率
//    Q(2,2)     = 1.0;//角度偏差
//    Q(3,3)     = 1.0;//角度偏差变化率
//    R          = 4.0;//车头偏向
//  }
//  else
//  {
//    Q(0,0)     = 8.0;//横向偏差1.5
//    Q(1,1)     = 1.0;//横向偏差变化率
//    Q(2,2)     = 1.5;//角度偏差
//    Q(3,3)     = 1.0;//角度偏差变化率
//    R          = 2.0;//车头偏向
//  }
  Q(0,0)     = 10.0;//横向偏差1.5
  Q(1,1)     = 2.0;//横向偏差变化率
  Q(2,2)     = 1.8;//角度偏差
  Q(3,3)     = 1.5;//角度偏差变化率
  R          = 5.0;//车头偏向
  //#######################################################
  // gain of lqr
//  auto t1 = std::chrono::high_resolution_clock::now();
  RowVector4f K = dlqr(A, B, Q, R);
//  auto t2 = std::chrono::high_resolution_clock::now();
//  std::chrono::duration<double> delta_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
//  std::cout<<"LQR1优化耗时为:"<<delta_time.count()<<std::endl;
  //计算偏差变化率
  Poi_f error_rate = ComputeErrorRate(current_vel.steer_angle,th_e,current_state.v);
  //状态量包括y向偏差，y向偏差变化率，角度偏差，角度偏差变化率，理想与实际速度偏差
  Vector4f x = Vector4f::Zero();
  x(0) = e;
  x(1) = error_rate[0]/DT;
  x(2) = th_e;
  x(3) = error_rate[1]/DT;

  bz_robot::MessageToRos::GetInstance()->VisualTargetPoint(cx[point],cy[point],cyaw[point]);
  //凸面的定义是：过面上任意一点做切面,表面总是在切面的下方,凸面曲率为正,一般是前进方向左正右负
  float delta = angles::normalize_angle(-K * x);
  //std::cout<<"横向偏差="<<e<<"角度偏差="<<th_e<<"车头偏角="<<delta<<std::endl;
  //std::cout<<"实际转角="<<current_vel.steer_angle<<std::endl;
  //std::cout<<"横向偏差="<<x(0)<<"/角度偏差="<<x(2)<<"/LQR偏转角="<<delta<<"/前瞻偏转角="<<forward_delta<<std::endl;
  //发布横向偏差，角度偏差，LQR转角，前馈转角
  delta = delta + forward_delta;
  clamp(delta,-param_.max_steer_angle,param_.max_steer_angle);

  control.velocity = tv;
  control.steer_angle = delta;
  //当发现需要过小半径圆弧时
  int point2          = ind + 50;
  if(point2 <= max_point)
  {
    //计算1m长度下的偏向角变化
    float delta_yaw = std::fabs(cyaw.at(ind) - cyaw.at(point2));
    if(delta_yaw>0.2)
    {
      std::cout<<"降速通过"<<std::endl;
      std::cout<<"降速前速度为："<<control.velocity<<"/车头偏向角为："<<control.steer_angle<<std::endl;
      control.velocity = std::max(0.2/delta_yaw,0.5)*control.velocity;
      control.velocity = std::max((float)0.3,control.velocity);
      control.steer_angle = control.steer_angle*std::min(1.4,delta_yaw/0.2);
      std::cout<<"降速后速度为："<<control.velocity<<"/车头偏向角为："<<control.steer_angle<<std::endl;
    }
  }
  clamp(control.velocity,current_vel.velocity-param_.max_acc*param_.dt,current_vel.velocity+param_.max_acc*param_.dt);
  clamp(control.steer_angle,current_vel.steer_angle-param_.max_steer_angle*param_.dt,current_vel.steer_angle+param_.max_steer_angle*param_.dt);
  clamp(control.velocity,param_.initial_vel,tv);
  //转化到机器人后轮中心
  control.velocity = ToBase(control.velocity,control.steer_angle);
  return control;
}
float LQR::ComputeLateralError(const State &state, const Vec_f &cx, const Vec_f &cy, const Vec_f &cyaw, int &nearest_id)
{
  float mind = std::numeric_limits<float>::max();
  int size = cx.size();
  for(int i=0; i<size; i++)
  {
    float idx = cx[i] - state.x;
    float idy = cy[i] - state.y;
    float d_e = idx*idx + idy*idy;

    if (d_e<mind){
      mind = d_e;
      nearest_id = i;
    }
  }
  if(nearest_id > size-1)
  {
    nearest_id = size-1;
  }
  if(nearest_id < 1)
  {
    nearest_id = 1;
  }
  //计算横向偏差
  Poi_f current_pose;
  current_pose[0] = state.x;
  current_pose[1] = state.y;
  Poi_f start_pose;
  start_pose[0] = cx[nearest_id-1];
  start_pose[1] = cy[nearest_id-1];
  Poi_f end_pose;
  end_pose[0] = cx[nearest_id];
  end_pose[1] = cy[nearest_id];
  Poi_f foot_point = ComputeFootPrint(current_pose,start_pose,end_pose);
  mind = std::sqrt(pow(current_pose[0] - foot_point[0],2) + pow(current_pose[1] - foot_point[1],2));
  //std::cout<<"×××××××××××××××最近点距离×××××××××××××××××××："<<mind<<std::endl;

  float dxl = cx[nearest_id] - state.x;
  float dyl = cy[nearest_id] - state.y;
  float angle = angles::normalize_angle(cyaw[nearest_id] - std::atan2(dyl, dxl));
  if (angle < 0) mind = mind * -1;
  //往前看0.4m,选择此刻的角度
//  nearest_id = nearest_id+30;
//  if(nearest_id > size-1)
//  {
//    nearest_id = size-1;
//  }
//  if(nearest_id < 1)
//  {
//    nearest_id = 1;
//  }
  return mind;
}
//todo为克服lqr本身在预测方面的缺点，增加一点儿
//float LQR::ComputeLateralError(const State &state, const Vec_f &cx, const Vec_f &cy, const Vec_f &cyaw, int &nearest_id)
//{
//  float mind = std::numeric_limits<float>::max();
//  State s1(0.0,0.0,0.0,0.0),s2(0.0,0.0,0.0,0.0),s(0.0,0.0,0.0,0.0);//后两个时刻的状态以及三个时刻综合后的状态
//  //1.预测后两个时刻的状态
//  UpdateState(last_lqr_result_[1],state,s1);
//  UpdateState(last_lqr_result_[1],s1,s2);
//  s.x   = 0.1*state.x + 0.2*s1.x + 0.7*s2.x;
//  s.y   = 0.1*state.y + 0.2*s1.y + 0.7*s2.y;
//  s.yaw = 0.1*state.yaw + 0.2*s1.yaw + 0.7*s2.yaw;
//  //2.找到混合后最近点相关误差
//  for(int i=0; i<cx.size(); i++)
//  {
//    float idx = cx[i] - s.x;
//    float idy = cy[i] - s.y;
//    float d_e = idx*idx + idy*idy;

//    if (d_e<mind){
//      mind = d_e;
//      nearest_id = i;
//    }
//  }
//  mind = std::sqrt(mind);
//  //std::cout<<"×××××××××××××××最近点距离×××××××××××××××××××："<<mind<<std::endl;

//  float dxl = cx[nearest_id] - s.x;
//  float dyl = cy[nearest_id] - s.y;
//  float angle = angles::normalize_angle(cyaw[nearest_id] - std::atan2(dyl, dxl));
//  if (angle < 0) mind = mind * -1;

//  return mind;
//}
//float LQR::ComputeLateralError(const State &state, const Vec_f &cx, const Vec_f &cy, const Vec_f &cyaw, int &nearest_id)
//{
//  float mind = std::numeric_limits<float>::max();
//  for(int i=0; i<cx.size(); i++)
//  {
//    float idx = cx[i] - state.x;
//    float idy = cy[i] - state.y;
//    float d_e = idx*idx + idy*idy;

//    if (d_e<mind){
//      mind = d_e;
//      nearest_id = i;
//    }
//  }
//  mind = std::sqrt(mind);
//  float dxl = cx[nearest_id] - state.x;
//  float dyl = cy[nearest_id] - state.y;
//  float angle = angles::normalize_angle(cyaw[nearest_id] - std::atan2(dyl, dxl));
//  //小车位于线路左侧大于0，位于右侧小于0
//  if (angle < 0) mind = mind * -1;
//  std::cout<<"×××××××××××××××最近点距离×××××××××××××××××××："<<mind<<std::endl;
//  return mind;
//}

void LQR::UpdateState(float &delta, const State &state_in, State &state_out)
{
  if (delta >= 20.0/180*M_PI) delta = 20.0/180*M_PI;
  if (delta <= - 20.0/180*M_PI) delta = - 20.0/180*M_PI;

  state_out.x = state_in.x + state_in.v * std::cos(state_in.yaw) * DT;
  state_out.y = state_in.y + state_in.v * std::sin(state_in.yaw) * DT;
  state_out.yaw = state_in.yaw + state_in.v / 0.64 * std::tan(delta) * DT;
  state_out.v = state_in.v;
}

Poi_f LQR::ComputeFootPrint(const Poi_f &cur_pose, const Poi_f &a, const Poi_f &b)
{
  Poi_f point;
  Poi_f a_p;
  a_p[0] = cur_pose[0] - a[0];
  a_p[1] = cur_pose[1] - a[1];
  Poi_f a_b;
  a_b[0] = b[0] - a[0];
  a_b[1] = b[1] - a[1];
  float len_square = a_b[0] * a_b[0] + a_b[1] * a_b[1];
  float ab_ap_product = a_b[0] * a_p[0] + a_b[1] * a_p[1];
  //distance表示的是起点与垂足点的连线向量比上起点与终点向量的比值，同向为正，异向为负
  float distance = ab_ap_product / len_square;
  if(distance < 0)
  {
      point = a;
  }
  else if(distance > 1)
  {
      point = b;
  }
  else
  {
      point = a_b;
      point[0] *= distance;
      point[1] *= distance;
      point[0] = a[0] + point[0];
      point[1] = a[1] + point[1];
  }
  return point;
}

Poi_f LQR::ComputeErrorRate(const float &delta, const float &yaw_error, const float &vel)
{
  Poi_f error_rate;
  //1.计算机器人横摆角速度
  float omiga = vel*delta/L;
  //2.计算横向偏差变化率
  error_rate[0] = vel*DT*sin(yaw_error-omiga*DT*0.5);
  //3.计算角度偏差变化率
  error_rate[1] = omiga*DT;
  //std::cout<<"横向偏差变化率："<<error_rate[0]<<"/角度偏差变化率："<<error_rate[1]<<std::endl;
  return error_rate;
}

}
