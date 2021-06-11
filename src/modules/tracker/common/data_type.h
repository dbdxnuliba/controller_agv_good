#pragma once

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>
#include<cmath>
#include<Eigen/Eigen>
#include "data_types.h"

namespace bz_robot
{
using Vec_f=std::vector<float>;
using Poi_f=std::array<float, 2>;
using Vec_Poi=std::vector<Poi_f>;

using Matrix4f = Eigen::Matrix<float, 4, 4>;
using RowVector4f = Eigen::Matrix<float, 1, 4>;
using Vector4f = Eigen::Vector4f;
struct State
{
  float x;
  float y;
  float yaw;
  float v;
  State(float x_, float y_, float yaw_, float v_){
    x = x_;
    y = y_;
    yaw = yaw_;
    v = v_;
  }
};
struct ControlState
{
  float lateral_error;//横向偏差
  float yaw_error;//角度偏差
  float mixed_error;//混合偏差
  float mixed_error_rate;//偏差变化率
};
struct BaseParam
{
  float y_error_max         = 0.5;//允许的最大横向偏差，大于该范围用阈值进行处理
  float yaw_error_max       = 0.5;//允许的最大角度偏差，大于该范围用阈值进行处理
  float error_max           = 0.45;//允许的最大综合偏差，大于该范围用阈值进行处理
  float d_error_max         = 0.45;//允许的最大综合偏差变化率，大于该范围用阈值进行处理
  float d_steer_angle_max   = 0.01;//允许的最大转角变化率，大于该范围用阈值进行处理
  float max_steer_angle = 0.349;
  float max_vel         = 1.0;
  float initial_vel     = 0.2;
  float max_acc         = 1.0;
  float dt              = 0.04;
  float wheel_distance  = 0.64;
};

inline Vec_f vec_diff(Vec_f input)
{
  Vec_f output;
  for(unsigned int i=1; i<input.size(); i++){
    output.push_back(input[i] - input[i-1]);
  }
  return output;
}

inline Vec_f cum_sum(Vec_f input)
{
  Vec_f output;
  float temp = 0;
  for(unsigned int i=0; i<input.size(); i++){
    temp += input[i];
    output.push_back(temp);
  }
  return output;
}

inline void smooth_yaw(Vec_f& cyaw)
{
  for(unsigned int i=0; i<cyaw.size()-1; i++){
    float dyaw = cyaw[i+1] - cyaw[i];

    while (dyaw > M_PI/2.0){
      cyaw[i+1] -= M_PI*2.0;
      dyaw = cyaw[i+1] - cyaw[i];
    }
    while (dyaw < -M_PI/2.0){
      cyaw[i+1] += M_PI*2.0;
      dyaw = cyaw[i+1] - cyaw[i];
    }
  }
}

template <typename T>
void inline clamp(T &x,const T &x_min,const T &x_max)
{
  if(x<x_min)
  {
    x = x_min;
  }
  else if(x>x_max)
  {
    x = x_max;
  }
  else
  {
    x = x;
  }
}
}
