#pragma once
#include "common/common.h"
#include "common/geometry.h"
#include <mutex>
#include <iostream>
namespace bz_robot {
class TrajectoryGenerator
{
public:
  TrajectoryGenerator();
  ~TrajectoryGenerator();
  void GenerateLinePath();
  void GenerateCirclePath();
  void GenerateCurvePath();
  Msg<PathData> GetPath();
private:
  Msg<PathData> path_;
  Pose<FLOAT_T> robot_pose_;//为简化计算，机器人初始值暂时定（0，0，0）
  std::mutex path_mutex_;
};
}

