/*************************************************************************
  > File Name: cubic_spline.h
  > Author: pan.cao
  > Mail: umrobot
  > Created Time:  8 27 2020
 ************************************************************************/
#pragma once
#include<iostream>
#include<vector>
#include<array>
#include<string>
#include<Eigen/Eigen>
#include<stdexcept>
#include "cubic_spline.h"
#include "geometry.h"
namespace bz_robot
{
struct SplineParam
{
  Vec_f x;
  Vec_f y;
  Vec_f yaw;
  Vec_f curvature;
  Vec_f s;
  Vec_f allowed_speed;//当前线路允许的最大速度
};
class Spline2D{
public:

  Spline2D();
  ~Spline2D();
  /**
     * @brief 将传入的路径处理成样条曲线
     * @param 输入参数:路径规划传入的路径,机器人当前位置
     * @param 输出参数:无
     * @return 返回值:无
     */
  void DealWithPath(const std::vector<Pose<float> > &path, const float &currnt_vel);
  /**
     * @brief 根据传入的弧长,计算出对应的坐标值
     * @param 输入参数:路径长度s
     * @param 输出参数:无
     * @return 返回值:坐标点
     */
  Poi_f GetPosition(float s);
  /**
     * @brief 根据传入的弧长,计算出对应的曲率
     * @param 输入参数:路径长度s
     * @param 输出参数:无
     * @return 返回值:曲率
     */
  double CalculateCurvature(float s);
  /**
     * @brief 根据传入的弧长,计算出对应点的偏向角
     * @param 输入参数:路径长度s
     * @param 输出参数:无
     * @return 返回值:偏航角
     */
  double CalculateYaw(float s);
  /**
     * @brief 根据传入的状态向量,结合样条曲线计算结果,计算最近点
     * @param 输入参数:机器人状态向量state,上次到达的最近点last_index
     * @param 输出参数:无
     * @return 返回值:偏航角
     */
  int CalculateNearestIndex(State state);
  /**
     * @brief 计算每条线路的允许速度
     * @param 输入参数:坐标位置x的容器,坐标位置y的容器,坐标yaw角的容器,线路长度s对应的容器,线路曲率k对应的容器,允许的目标速度
     * @param 输出参数:路线速度的集合
     * @return 返回值:无
     */
  Vec_f CalculateSpeedProfile(const Vec_f &rs, const Vec_f &rk, float target_speed, const float &current_vel);

  inline SplineParam GetTrack(){return spline_;}
  inline void ClearSpline(){  spline_.x.clear();
                              spline_.y.clear();
                              spline_.yaw.clear();
                              spline_.s.clear();
                              spline_.curvature.clear();
                              spline_.allowed_speed.clear();}
  //获取线路全长
  inline double GetWholeLength()
  {
    return spline_.s.at(spline_.s.size() - 1);
  }
  /**
   * @brief 结构体对机器人的状态量和控制输入进行赋值
   * @param
   * @param
   * @return
   */
  bool IsGoReached(const State &x);
private:
  /**
     * @brief 根据输入的路径,计算各个路径点之前的总弧长集合
     * @param 输入参数:坐标位置x的容器,坐标位置y的容器
     * @param 输出参数:无
     * @return 返回值:路线s的集合
     */
  Vec_f CalculateS(Vec_f x, Vec_f y);
  void ExtendPath(Vec_f &x, Vec_f &y, Vec_f &yaw);
private:
  Spline sx;
  Spline sy;
  Vec_f s;
  SplineParam spline_;//对生成的样条曲线进行采样,并记录采样点坐标的集合

  int target_point_index_;//路径规划要求的目标点编号,但是不一定是最后一个点,所以需要记下来

  Pose<double> target_point_;//目标点对应坐标
  BaseParam base_param_;
};
}
