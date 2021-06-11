#pragma once
#include <matplotlib/matplotlibcpp.h>
#include <vector>
#include "common/common.h"
#include "common/geometry.h"
namespace bz_robot {
class MatlabPlot
{
public:
  MatlabPlot();
  ~MatlabPlot();
  void PlotPathAndRobot(const std::vector<Pose<FLOAT_T>> &robot_pose_list,const std::vector<Pose<FLOAT_T>> &path_list);
};
}

