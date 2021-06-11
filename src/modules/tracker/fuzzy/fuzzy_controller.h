#pragma once
#include <iostream>
#include <string>
#include <math.h>
#include "deal_with_track.h"
#include "frenet_connect_cartesian.h"
#include <memory>
#include <angle.h>
#include "controller_to_ros.h"
#include "fl/Headers.h"
#include "common/print.h"

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

using namespace std;
using namespace fl;
namespace bz_robot {

class FuzzyPID
{
public:
  FuzzyPID();
  ~FuzzyPID();
public:
  void SetTrack(const PathData &path, const ControlData &data);
  void ClearTrack();
  std::vector<Pose<FLOAT_T> > GetTrackNew();
  //由于接收到的机器人速度为后轮速度，运算过程中需要的可能是机器人中心速度，所以需要修改
  ControlData RunFuzzy(const Pose<FLOAT_T> &state, const ControlData &current_vel,Msg<Pose<FLOAT_T>> &goal);
private:
  /**
     * @brief 初始化模糊控制器
     * @param 输入参数:无
     * @param 输出参数:无
     * @return 返回值:无
     */
  void Init();
  /**
     * @brief 为降低模糊控制的维度，减少规则数量，可以将横向偏差和角度偏差融合为综合偏差
     * @param 输入参数:横向偏差，角度偏差
     * @param 输出参数:无
     * @return 返回值:综合偏差
     */
  float SynthesizedError(float &lateral_error, float &angle_error);
  /**
     * @brief 实现模糊控制
     * @param 输入参数:混合偏差,偏差变化率
     * @param 输出参数:无
     * @return 返回值:车头角变化
     */
  float RealizeFuzzy(const float &error,const float &d_error);
  /**
     * @brief 因为定位结果跳动，导致通过两次的偏差求解偏差变化率不准确，需要通过机器人实际的运动学参数进行估计
     * @param 输入参数:车头偏向角delta，角度偏差，机器人中心速度
     * @param 输出参数:无
     * @return 返回值:偏差变化率
     */
  float ComputeErrorRate(const float &delta,const float &yaw_error,const float &vel);
  float inline FromBase(const float &vs,const float &delta)
  {
   return vs * std::sqrt(1 + 0.25 * std::pow(std::tan(delta),2));
  }
  float inline ToBase(const float &v, const float &u0)
  {
    return v / std::sqrt(1 + 0.25 * std::pow(std::tan(u0),2));
  }
private:
  Spline2D track_;
  float last_error_;
  FrenetHdr frenet_hdr_;
  const BaseParam param_;
  Eigen::MatrixXf fuzzy_relation_matrix_;
  std::map<int,Eigen::MatrixXf> m_fuzzy_relation_matrix_;

  float last_forward_delta_;

  Engine* engine_;//新建模糊工程
  InputVariable* input_error_;//输入偏差
  InputVariable* input_error_rate_;//输入偏差变化率
  OutputVariable* output_delta_;//对应的机器人输出
  RuleBlock* mamdani_;//模糊推理规则
};
}
