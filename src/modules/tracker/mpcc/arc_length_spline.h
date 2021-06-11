// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#ifndef MPCC_ARC_LENGTH_SPLINE_H
#define MPCC_ARC_LENGTH_SPLINE_H

#include "cubic_spline.h"
#include "types.h"
#include "params.h"
#include <map>
namespace mpcc{
//return value
struct RawPath{
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
};
// data struct
//路径离散化后的点坐标以及前i个点对应的总弧长
struct PathData{
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXd s;
    int n_points;
};

class ArcLengthSpline {
public:
  // X and Y spline used for final spline fit
  void gen2DSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y);
  Eigen::Vector2d getPostion(double) const;
  Eigen::Vector2d getDerivative(double) const;
  Eigen::Vector2d getSecondDerivative(double) const;
  double getLength() const;
  double porjectOnSpline(const State &x) const;
  /**
   * @brief 结构体对机器人的状态量和控制输入进行赋值
   * @param
   * @param
   * @return
   */
  bool IsGoReached(const State &x);
  /**
   * @brief 结构体对机器人的状态量和控制输入进行赋值
   * @param
   * @param
   * @return
   */
  int FindNearestPoint(const Pose<double> &robot_pose);
  /**
   * @brief 结构体对机器人的状态量和控制输入进行赋值
   * @param
   * @param
   * @return
   */
  bool IsPassedPoint(const int &index,const Pose<double> &robot_pose);
  ArcLengthSpline();
  ArcLengthSpline(const PathToJson &path);
  double ComputeCurvature(const double &s);
private:
  //void setData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in);
  void setRegularData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in,const Eigen::VectorXd &s_in);
  Eigen::VectorXd compArcLength(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in) const;
  PathData resamplePath(const CubicSpline &initial_spline_x,const CubicSpline &initial_spline_y,double total_arc_length) const;
  RawPath outlierRemoval(const Eigen::VectorXd &X_original,const Eigen::VectorXd &Y_original) const;
  void fitSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y);
  double unwrapInput(double x) const;
  RawPath extendPath(const RawPath &path);
private:
  PathData path_data_;      //initial data and data used for successive fitting
  CubicSpline spline_x_;
  CubicSpline spline_y_;
  Param param_;

  int target_point_index_;//路径规划要求的目标点编号,但是不一定是最后一个点,所以需要记下来
  Pose<double> target_point_;//目标点对应坐标
};
}
#endif //MPCC_ARC_LENGTH_SPLINE_H
