#include "deal_with_track.h"
#include <float.h>
#include <angles/angles.h>
#include "controller_to_ros.h"
namespace bz_robot
{

Spline2D::Spline2D()
{
  target_point_index_ = 65535;
  target_point_.heading_angle = FLT_MAX;
  target_point_.position.x    = FLT_MAX;
  target_point_.position.y    = FLT_MAX;

  spline_.x.clear();
  spline_.y.clear();
  spline_.yaw.clear();
  spline_.s.clear();
  spline_.curvature.clear();
  spline_.allowed_speed.clear();
}

Spline2D::~Spline2D()
{

}

void Spline2D::DealWithPath(const std::vector<Pose<float>> &path, const float &currnt_vel)
{
  //1.接受主函数传入的路径,为对路线进行样条插值做准备
  Vec_f path_x;
  Vec_f path_y;
  Vec_f path_yaw;
  path_x.clear();
  path_y.clear();
  path_yaw.clear();

  for(int i=0;i<path.size();i++)
  {
    path_x.push_back(path.at(i).position.x);
    path_y.push_back(path.at(i).position.y);
    path_yaw.push_back(path.at(i).heading_angle);
  }
//  std::cout<<"###########打印处理前的线路#############"<<std::endl;
//  for(int i=0;i<path_x.size();i++)
//  {
//    std::cout<<"点号："<<i<<std::endl;
//    std::cout<<"path x"<<path_x.at(i)<<std::endl;
//    std::cout<<"path y"<<path_y.at(i)<<std::endl;
//    if(i!=0)
//    {
//      std::cout<<"path theta"<<atan2(path_y.at(i)-path_y.at(i-1),path_x.at(i)-path_x.at(i-1))<<std::endl;
//    }
//    std::cout<<" "<<std::endl;
//  }
  //对路线进行延长
  target_point_.position.x = path_x.at(path_x.size()-1);
  target_point_.position.y = path_y.at(path_y.size()-1);
  target_point_.heading_angle = path_yaw.at(path_yaw.size()-1);
  ExtendPath(path_x,path_y,path_yaw);
  //2.计算样条曲线相关参数,s长度从0开始
  s = CalculateS(path_x, path_y);
  sx = Spline(s, path_x);
  sy = Spline(s, path_y);
  //3.对样条曲线结果进行等距采样,并保存采样点,采样距离0.02m
  spline_.x.clear();
  spline_.y.clear();
  spline_.yaw.clear();
  spline_.s.clear();
  spline_.curvature.clear();
  spline_.allowed_speed.clear();
  //std::cout<<"################################"<<std::endl;
  for(float i=0; i<s.back(); i+=0.02){
    std::array<float, 2> point = GetPosition(i);
    spline_.x.push_back(point[0]);
    spline_.y.push_back(point[1]);
    //spline_.yaw.push_back(CalculateYaw(i));
    spline_.curvature.push_back(CalculateCurvature(i));
    spline_.s.push_back(i);
    //std::cout<<"Curvature:"<<CalculateCurvature(i)<<std::endl;
  }
  //3.1对最后一个点进行特殊处理
  std::array<float, 2> end_point = GetPosition(s.back());
  spline_.x.push_back(end_point[0]);
  spline_.y.push_back(end_point[1]);
  spline_.yaw.push_back(CalculateYaw(s.back()));
  spline_.curvature.push_back(CalculateCurvature(s.back()));
  spline_.s.push_back(s.back());
  //todo 另一種求線路朝向角的計算
  //******************************************
  spline_.yaw.clear();
  for(int i=0;i<spline_.x.size()-1;i++)
  {
    float yaw = atan2(spline_.y.at(i+1)-spline_.y.at(i),spline_.x.at(i+1)-spline_.x.at(i));
    spline_.yaw.push_back(yaw);
  }
  spline_.yaw.push_back(CalculateYaw(s.back()));
  //******************************************
  //controller::MessageToRos::GetInstance()->VisuallocalPath(spline_.x,spline_.y,spline_.yaw);
  //4.估算各段允许的最大速度
  spline_.allowed_speed = CalculateSpeedProfile(spline_.s,spline_.curvature,1.0,currnt_vel);
//  std::cout<<"#########打印处理后的线路################"<<std::endl;
//  for(int i=0;i<spline_.x.size();i++)
//  {
//    std::cout<<"点号："<<i<<std::endl;
//    std::cout<<"spline_.x"<<spline_.x.at(i)<<std::endl;
//    std::cout<<"spline_.y"<<spline_.y.at(i)<<std::endl;
//    std::cout<<"spline_.yaw="<<spline_.yaw.at(i)<<std::endl;
//    std::cout<<"spline_.s"<<spline_.s.at(i)<<std::endl;
//    std::cout<<"spline_.curvature"<<spline_.curvature.at(i)<<std::endl;
//    std::cout<<"spline_.allowed_speed"<<spline_.allowed_speed.at(i)<<std::endl;
//    std::cout<<" "<<std::endl;
//  }
//  for(int i=0;i<spline_.x.size()-1;i++)
//  {
//    float a = angles::normalize_angle(spline_.yaw.at(i+1)-spline_.yaw.at(i));
//    if(std::fabs(a)>0.05)
//    {
//      std::cout<<"点之间的角度偏差="<<a<<std::endl;
//    }
//  }
  //5.找到停车点
  //找到停车点,
  float dis = FLT_MAX;
  float dx  = 0.0;
  float dy  = 0.0;
  for(int i=spline_.x.size()-1;i>=0;i--)
  {
    dx = target_point_.position.x -  spline_.x.at(i);
    dy = target_point_.position.y -  spline_.y.at(i);

    if((dx*dx + dy*dy)<dis)
    {
      target_point_index_ = i;
      dis = dx*dx + dy*dy;

    }
    else
    {
      //std::cout<<" dis ="<< dis<<std::endl;
      target_point_index_ = i;
      break;
    }
  }
}

Poi_f Spline2D::GetPosition(float s)
{
  float x = sx.CalculatePositionFromS(s);
  float y = sy.CalculatePositionFromS(s);
  return {{x, y}};
}

double Spline2D::CalculateCurvature(float s)
{
  float k = 0.0;
  float dx = sx.CalculateDrivate(s);
  float ddx = sx.CalculateSecondDrivate(s);
  float dy = sy.CalculateDrivate(s);
  float ddy = sy.CalculateSecondDrivate(s);
  k = (ddy * dx - ddx * dy)/std::pow((dx * dx + dy * dy),3/2);
  //std::cout<<"k="<<k<<std::endl;
  //return (ddy * dx - ddx * dy)/(dx * dx + dy * dy);
  return std::fabs(k);
}

double Spline2D::CalculateYaw(float s)
{
  float dx = sx.CalculateDrivate(s);
  float dy = sy.CalculateDrivate(s);
  return std::atan2(dy, dx);
}

int Spline2D::CalculateNearestIndex(State state)
{
  float dis = FLT_MAX;
  float index = 0;
  for(int i=0; i<spline_.x.size(); i++)
  {
    float idx = spline_.x[i] - state.x;
    float idy = spline_.y[i] - state.y;
    float d_e = idx*idx + idy*idy;

    if (d_e<dis)
    {
      dis = d_e;
      index = i;
    }
  }
  return index;
}

Vec_f Spline2D::CalculateSpeedProfile(const Vec_f &rs, const Vec_f &rk, float target_speed, const float &current_vel)
{
  Vec_f speed_profile(rs.size(), target_speed);

  for(int i=0; i < rs.size(); i++){
    //终止阶段对线路允许的最大速度进行处理
    if(GetWholeLength() - rs.at(i) <= 2.0)
    {
      target_speed = (GetWholeLength() - rs.at(i))*0.5;
    }
    else if(rs.at(i) <= 2.0)
    {
      target_speed = current_vel + rs.at(i)*0.5;
    }
    else
    {
      target_speed = base_param_.max_vel;
    }
    clamp(target_speed,base_param_.initial_vel,base_param_.max_vel);
    //根据曲率增加速度衰减系数
    //target_speed *= 1/std::exp(std::fabs(rk.at(i)));
    speed_profile[i] = target_speed;
  }
  return speed_profile;
}

bool Spline2D::IsGoReached(const State &x)
{
  //如果路径为空，则直接返回true
  if(spline_.x.size()==0)
  {
    std::cout<<"没有需要跟随的线路："<<std::endl;
    return true;
  }
  int nearest_index = CalculateNearestIndex(x);
  //std::cout<<"nearest_index="<<nearest_index<<"  "<<"target_point_index_="<<target_point_index_<<"  "<<"spline_.x.size()="<<spline_.x.size()<<std::endl;
  if(nearest_index >= target_point_index_)
  {
    //std::cout<<"到终点的距离："<<std::sqrt(pow(spline_.x[nearest_index]-x.x,2)+pow(spline_.y[nearest_index]-x.y,2))<<std::endl;
    target_point_index_ = 65535;
    target_point_.heading_angle = FLT_MAX;
    target_point_.position.x    = FLT_MAX;
    target_point_.position.y    = FLT_MAX;
    return true;
  }
  return false;
}

Vec_f Spline2D::CalculateS(Vec_f x, Vec_f y)
{
  Vec_f ds;
  Vec_f out_s{0};
  Vec_f dx = vec_diff(x);//dx[0]有长度,不为0
  Vec_f dy = vec_diff(y);//dy[0]有长度,不为0
  //1.获取路径点之间的长度容器
  for(unsigned int i=0; i<dx.size(); i++){
    ds.push_back(std::sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
  }
  //2.输出前i个点对应弧长的容器
  Vec_f cum_ds = cum_sum(ds);
  //out_s长度从零开始
  out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
  return out_s;
}

void Spline2D::ExtendPath(Vec_f &x, Vec_f &y, Vec_f &yaw)
{
  if(x.size() < 2)
  {
    std::cout<<"点太少,暂时不需要处理"<<std::endl;
    return ;
  }
//  if(x.size() == 2)
//  {
    double x0,y0,x1,y1,theta;
    int size = x.size();

    x1 = x.at(size-1);
    y1 = y.at(size-1);
    theta = yaw.at(size-1);

    x0     = x1 + 0.05*std::cos(theta);
    y0     = y1 + 0.05*std::sin(theta);

    x.push_back(x0);
    y.push_back(y0);
//  }
//  else
//  {
//    double theta,theta1,theta2;
//    double x0,y0,x1,y1,x2,y2,x3,y3;
//    int size = x.size();

//    x1 = x.at(size-1);
//    y1 = y.at(size-1);

//    x2 = x.at(size-2);
//    y2 = y.at(size-2);

//    x3 = x.at(size-3);
//    y3 = y.at(size-3);

//    theta1 = atan2(y1 -y2,x1 -x2);
//    theta2 = atan2(y2 -y3,x2 -x3);

//    theta  = angles::normalize_angle(theta1 + (theta1 - theta2));
//    x0     = x1 + 0.05*std::cos(theta);
//    y0     = y1 + 0.05*std::sin(theta);

//    x.push_back(x0);
//    y.push_back(y0);
//  }
  return;
}

}
