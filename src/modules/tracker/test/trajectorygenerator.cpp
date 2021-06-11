#include "trajectorygenerator.h"
namespace bz_robot {
TrajectoryGenerator::TrajectoryGenerator()
{
  path_.data.clear();
}

TrajectoryGenerator::~TrajectoryGenerator()
{

}

void TrajectoryGenerator::GenerateLinePath()
{
  std::lock_guard<std::mutex> guard(path_mutex_);
  Pose<FLOAT_T> middle_pose;
  for(int i=0;i<=50;i++)
  {
    middle_pose.position.x = 0.2*i;
    middle_pose.position.y = 1.0;
    middle_pose.heading_angle = 0.0;
    path_.data.push_back(middle_pose);
  }
}

void TrajectoryGenerator::GenerateCirclePath()
{
  std::lock_guard<std::mutex> guard(path_mutex_);
  float radius =3.0;
  Pose<FLOAT_T> middle_pose;
  for(int i=0;i<std::floor(1.5*M_PI*20);i++)
  {
    middle_pose.position.x = radius*cos(-0.5*M_PI+0.05*i);
    middle_pose.position.y = radius + radius*sin(-0.5*M_PI+0.05*i);
    middle_pose.heading_angle = 0.05*i;
    path_.data.push_back(middle_pose);
  }
}

void TrajectoryGenerator::GenerateCurvePath()
{
  std::lock_guard<std::mutex> guard(path_mutex_);
  Pose<FLOAT_T> middle_pose;
  for(int i=0;i<std::floor(15*M_PI*20);i++)
  {
    middle_pose.position.x = 0.05*i;
    middle_pose.position.y = 5*sin(0.05*i/5);
    middle_pose.heading_angle = cos(0.05*i);
    path_.data.push_back(middle_pose);
  }
}

Msg<PathData> TrajectoryGenerator::GetPath()
{
  std::lock_guard<std::mutex> guard(path_mutex_);
  return path_;
}
}

