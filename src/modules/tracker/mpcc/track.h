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
#pragma once

#include "config.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <nlohmann/json.hpp>

namespace mpcc {
//used namespace
using json = nlohmann::json;

struct TrackPos {
  const Eigen::VectorXd X;
  const Eigen::VectorXd Y;
  const Eigen::VectorXd Theta;

  const Eigen::VectorXd X_inner;
  const Eigen::VectorXd Y_inner;
  const Eigen::VectorXd Theta_inner;

  const Eigen::VectorXd X_outer;
  const Eigen::VectorXd Y_outer;
  const Eigen::VectorXd Theta_outer;
};

class Track {
public:
  Track();
  ~Track();
  /**
   * @brief  便于其他程序获取路径点
   * @param  输入参数:无
   * @param  输出参数:无
   * @return 返回值:路径点信息
   */
  TrackPos GetTrack();
  /**
   * @brief  从线程程序中获取路径点,转化为另一种格式
   * @param  输入参数:路径点信息
   * @param  输出参数:无
   * @return 返回值:无
   */
  void SetTrack(const std::vector<Pose<double>> &path);
private:
  /**
   * @brief 对路径进行三次样条插值
   * @param 输入参数:路径点xy坐标
   * @param 输出参数:无
   * @return 返回值:无
   */
  //void TransformTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y);
private:
  //中心路径的坐标集合
  Eigen::VectorXd X;
  Eigen::VectorXd Y;
  Eigen::VectorXd Theta;
  //内圈路径的坐标集合
  Eigen::VectorXd X_inner;
  Eigen::VectorXd Y_inner;
  Eigen::VectorXd Theta_inner;
  //外圈路径的坐标集合
  Eigen::VectorXd X_outer;
  Eigen::VectorXd Y_outer;
  Eigen::VectorXd Theta_outer;
};
}
