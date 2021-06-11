#include <trajectorygenerator.h>
#include <matlabplot.h>
#include "fuzzy_controller.h"
#include "lqr_speed_steer_control.h"
#include <ros/ros.h>
#include <thread>
using namespace bz_robot;
Pose<FLOAT_T> ComputeRobotPose(const Pose<FLOAT_T> &last_pose,const Msg<ControlData> &robot_vel)
{
  Pose<FLOAT_T> new_pose;
  //1.首先获得机器人中心的速度和角速度
  float v = robot_vel.data.velocity * std::sqrt(1 + 0.25 * std::pow(std::tan(robot_vel.data.steer_angle),2));
  float w = v*tan(robot_vel.data.steer_angle)/0.64;
  float beta = atan(0.5*tan(robot_vel.data.steer_angle));
  //2.根据上一时刻位置，估计新的机器人位置
  new_pose.heading_angle = last_pose.heading_angle + w*0.04;
  new_pose.position.x    = last_pose.position.x + v*0.04*cos(0.5*new_pose.heading_angle+0.5*last_pose.heading_angle+beta);
  new_pose.position.y    = last_pose.position.y + v*0.04*sin(0.5*new_pose.heading_angle+0.5*last_pose.heading_angle+beta);
  return new_pose;
}
int main(int argc,char** argv)
{
  //1.生成仿真轨迹
  ros::init(argc,argv,"test");
  TrajectoryGenerator t;
  t.GenerateLinePath();
  //t.GenerateCirclePath();
  //t.GenerateCurvePath();
  Msg<PathData> path = t.GetPath();
  Msg<Pose<FLOAT_T>> goal_pose;
  goal_pose.data= path.data.back();
  //2.循环进行机器人位置控制
  //std::shared_ptr<FuzzyPID> p_fuzzy = std::make_shared<FuzzyPID>();
  std::shared_ptr<LQR> p_lqr = std::make_shared<LQR>();
  Msg<ControlData> robot_vel;
  Pose<FLOAT_T> robot_pose(0.0,0.0,0.0);
  //用于存放用于绘图的数据
  std::vector<Pose<FLOAT_T> > robot_pose_list;
  std::vector<Pose<FLOAT_T> > path_list;
  robot_pose_list.clear();
  path_list.clear();
  robot_vel.data.velocity = 0.0;
  robot_vel.data.steer_angle = 0.0;
  //p_fuzzy ->SetTrack(path.data,robot_vel.data);
  p_lqr->SetTrack(path.data,robot_vel.data);
  ros::NodeHandle nh("~");
  ros::Rate r(25);
  while(ros::ok())
  {
    ros::spinOnce();
    //2.1 计算机器人位置
    robot_pose = ComputeRobotPose(robot_pose,robot_vel);
    //2.2 进行模糊控制
    //robot_vel.data = p_fuzzy->RunFuzzy(robot_pose,robot_vel.data,goal_pose);
    robot_vel.data = p_lqr->ComputeLqr(robot_pose,robot_vel.data,goal_pose);
    if(robot_vel.data.steer_angle==0.0&&robot_vel.data.velocity==0.0)
    {
      break;
    }
    robot_pose_list.push_back(robot_pose);
    r.sleep();
  }

  //3.绘制路径以及机器人位置形成轨迹
  //path_list = p_fuzzy->GetTrackNew();
  path_list = p_lqr->GetTrackNew();
  MatlabPlot plot;
  plot.PlotPathAndRobot(robot_pose_list,path_list);
  return 0;
}
