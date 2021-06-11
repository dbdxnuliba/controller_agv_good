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

#include "track.h"
namespace mpcc{
Track::Track()
{
  std::cout << "默认构造函数" << std::endl;
}

Track::~Track()
{
  std::cout << "默认析构函数" << std::endl;
}

TrackPos Track::GetTrack()
{
  return {X,Y,Theta,X_inner,Y_inner,Theta_inner,X_outer,Y_outer,Theta_outer};
}

void Track::SetTrack(const std::vector<Pose<double>> &path)
{
  int   size       = path.size();
  //1.获取中心路径坐标点
  X     = Eigen::VectorXd::Zero(size);
  Y     = Eigen::VectorXd::Zero(size);
  Theta = Eigen::VectorXd::Zero(size);
  for(int i=0;i<size;i++)
  {
    X(i)     = path.at(i).position.x;
    Y(i)     = path.at(i).position.y;
    Theta(i) = path.at(i).heading_angle;
  }
  //2.获取内侧边缘坐标点
  X_inner     = Eigen::VectorXd::Zero(size);
  Y_inner     = Eigen::VectorXd::Zero(size);
  Theta_inner = Eigen::VectorXd::Zero(size);
  //3.获取外侧边缘坐标点
  X_outer     = Eigen::VectorXd::Zero(size);
  Y_outer     = Eigen::VectorXd::Zero(size);
  Theta_outer = Eigen::VectorXd::Zero(size);
  //4.进行三次样条插值
  //TransformTrack(X,Y);
}

//void Track::TransformTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y)
//{
//  track_.gen2DSpline(X,Y);
//}
}
