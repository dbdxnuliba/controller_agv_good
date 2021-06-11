#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include<Eigen/Eigen>

namespace bz_robot
{
class MessageToRos;
using MessageHdr = std::shared_ptr<MessageToRos>;
using MessagePtr = MessageToRos*;
class MessageToRos
{
public:
  MessageToRos();
  ~MessageToRos();
  void RobotPosePublish(const float &pose_x,const float &pose_y,const float &pose_theta);
  void ComputeVelPublish(const float &vs,const float &delta);
  void ActualVelPublish(const float &vs,const float &delta);
  void FuzzyIoPublish(const float &error,const float &derror,const float &delta,const float &delta_new);
  void ErrorStatusPublish(const float &y_error,const float &th_error,const float &new_error,const float &new_error_rate,const float &steer);
  void MPCCostPublish(const float &x, const float &y,const float &z,const float &w);
  void VisualTargetPoint(const float &x, const float &y,const float &yaw);
  void VisualMpcPredictPoint(const std::vector<Eigen::Vector3f> &point);
  void VisuallocalPath(const std::vector<nav_msgs::Path> &paths);
  void VisuallocalPath(const std::vector<geometry_msgs::Point> &points);
  void DebugPublish(const float &x, const float &y,const float &z,const float &w);
  void VisualFootPoint(const float &x, const float &y, const float &yaw);
  static MessagePtr GetInstance()
  {
    static MessageToRos instance;
    return &instance;
  }
private:
  ros::Publisher compute_vel_pub_;//控制器计算出的速度信息
  ros::Publisher actual_vel_pub_;//按照周期分割后实际发送的速度
  ros::Publisher odom_pub_;//将收到的里程信息发布出来，观察延时情况
  ros::Publisher fuzzy_io_pub_;//模糊控制输入角度，横向偏差，输出的车头偏向角
  ros::Publisher error_status_pub_;//控制横向误差，横向误差变化率，角度误差，角度误差变化率
  ros::Publisher spline_param_pub_;//发布机器人角度，以及对应的线路角度
  ros::Publisher target_pose_pub_;//可视化跟踪目标点
  ros::Publisher prediction_point_pub_;//mpc預測點可視化
  ros::Publisher foot_point_pub_;
  ros::Publisher spline_pub_;
  ros::Publisher mpc_cost_pub_;//将状态量，控制量，控制变化率的代价值发布
  ros::Publisher some_variable_pub_;//临时增加，用于测量收到的速度，下发的速度，模糊计算得到的速度，经过过圆弧减速后的速度
};
}
