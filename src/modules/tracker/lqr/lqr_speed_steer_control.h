#pragma once

#include<iostream>
#include<limits>
#include<vector>

#include<sys/time.h>
#include "deal_with_track.h"
#include <angles/angles.h>
#include "controller_to_ros.h"

#define DT 0.04
#define L 0.64

namespace bz_robot {
struct LqrResult
{
  Vec_f result;
  int   result_num;
};
class LQR
{
public:
  LQR();
  ~LQR();
  ControlData ComputeLqr(const Pose<FLOAT_T> &state, const ControlData &current_vel,const Msg<Pose<FLOAT_T>> &goal);
  void SetTrack(const PathData &path, const ControlData &data);
  void ClearTrack();
  std::vector<Pose<FLOAT_T> > GetTrackNew();
private:
  Matrix4f SolveDare(Matrix4f A, Vector4f B, Matrix4f Q, float R);
  RowVector4f dlqr(Matrix4f A, Vector4f B, Matrix4f Q, float R);
  float ComputeLateralError(const State &state, const Vec_f &cx, const Vec_f &cy, const Vec_f &cyaw,int &nearest_id);
  /**
     * @brief 对机器人状态量进行更新
     * @param 输入参数:车头偏向角delta
     * @param 输出参数:状态量
     * @return 返回值:无
     */
  void UpdateState(float &delta,const State &state_in,State &state_out);
  Poi_f ComputeFootPrint(const Poi_f &cur_pose, const Poi_f &a,const Poi_f &b);
  /**
     * @brief 因为定位结果跳动，导致通过两次的偏差求解偏差变化率不准确，需要通过机器人实际的运动学参数进行估计
     * @param 输入参数:车头偏向角delta，角度偏差，机器人中心速度
     * @param 输出参数:无
     * @return 返回值:偏差变化率
     */
  Poi_f ComputeErrorRate(const float &delta,const float &yaw_error,const float &vel);
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
  VectorX2<float> last_lqr_result_;//记录上一次的求解结果
  const BaseParam param_;
};

}
