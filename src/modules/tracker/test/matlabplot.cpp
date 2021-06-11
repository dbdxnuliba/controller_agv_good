#include "matlabplot.h"
namespace bz_robot {
MatlabPlot::MatlabPlot()
{

}

MatlabPlot::~MatlabPlot()
{

}

void MatlabPlot::PlotPathAndRobot(const std::vector<Pose<FLOAT_T> > &robot_pose_list, const std::vector<Pose<FLOAT_T> > &path_list)
{
  std::vector<double> robot_x;
  std::vector<double> robot_y;
  std::vector<double> path_x;
  std::vector<double> path_y;

  for(int i=0;i<robot_pose_list.size();i++)
  {
    robot_x.push_back(robot_pose_list.at(i).position.x);
    robot_y.push_back(robot_pose_list.at(i).position.y);
  }

  for(int j=0;j<path_list.size();j++)
  {
    path_x.push_back(path_list.at(j).position.x);
    path_y.push_back(path_list.at(j).position.y);
  }
  matplotlibcpp::clf();
  matplotlibcpp::figure(1);
  matplotlibcpp::plot(robot_x,robot_y,"r--");
  matplotlibcpp::plot(path_x,path_y,"k-");
  matplotlibcpp::axis("equal");
  matplotlibcpp::xlabel("X [m]");
  matplotlibcpp::ylabel("Y [m]");
  matplotlibcpp::pause(22.0);
}
}

