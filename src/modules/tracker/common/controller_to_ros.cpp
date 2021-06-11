#include "controller_to_ros.h"
#include <angles/angles.h>
namespace bz_robot
{

MessageToRos::MessageToRos()
{
  ros::NodeHandle nh;
  odom_pub_        = nh.advertise<geometry_msgs::Vector3Stamped>("controller_odom", 1, true);
  compute_vel_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("controller_compute_vel", 1, true);
  actual_vel_pub_  = nh.advertise<geometry_msgs::Vector3Stamped>("controller_actual_vel", 1, true);
  fuzzy_io_pub_    = nh.advertise<geometry_msgs::QuaternionStamped>("controller_fuzzy_io", 1, true);
  error_status_pub_= nh.advertise<geometry_msgs::PoseStamped>("controller_error_status", 1, true);
  target_pose_pub_ = nh.advertise<visualization_msgs::Marker>("controller_target_pose", 1, true);
  prediction_point_pub_ = nh.advertise<visualization_msgs::Marker>("controller_prediction_point", 1, true);
  foot_point_pub_ = nh.advertise<visualization_msgs::Marker>("controller_foot_point", 1, true);
  spline_pub_           = nh.advertise<visualization_msgs::Marker>("controller_spline", 1, true);
  mpc_cost_pub_         = nh.advertise<geometry_msgs::QuaternionStamped>("controller_mpc_cost", 1, true);
  some_variable_pub_    = nh.advertise<geometry_msgs::QuaternionStamped>("controller_debug", 1, true);
}

MessageToRos::~MessageToRos()
{

}

void MessageToRos::RobotPosePublish(const float &pose_x, const float &pose_y, const float &pose_theta)
{
  geometry_msgs::Vector3Stamped msg;

  msg.header.stamp    = ros::Time::now();
  msg.vector.x = pose_x;
  msg.vector.y = pose_y;
  msg.vector.z = pose_theta;

  odom_pub_.publish(msg);
}

void MessageToRos::ComputeVelPublish(const float &vs, const float &delta)
{
  geometry_msgs::Vector3Stamped msg;

  msg.header.stamp = ros::Time::now();
  msg.vector.x     = vs;
  msg.vector.y     = delta;
  msg.vector.z     = 0.0;

  compute_vel_pub_.publish(msg);
}

void MessageToRos::ActualVelPublish(const float &vs, const float &delta)
{
  geometry_msgs::Vector3Stamped msg;

  msg.header.stamp = ros::Time::now();
  msg.vector.x     = vs;
  msg.vector.y     = delta;
  msg.vector.z     = 0.0;

  actual_vel_pub_.publish(msg);
}
//delta代表自编的模糊控制规则计算的结果，delta_new代表利用fuzzylite计算得到的结果
void MessageToRos::FuzzyIoPublish(const float &error, const float &derror, const float &delta, const float &delta_new)
{
  geometry_msgs::QuaternionStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.quaternion.x = error;
  msg.quaternion.y = derror;
  msg.quaternion.z = delta;
  msg.quaternion.w = delta_new;

  fuzzy_io_pub_.publish(msg);
}

void MessageToRos::ErrorStatusPublish(const float &y_error, const float &th_error, const float &new_error, const float &new_error_rate, const float &steer)
{
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.pose.orientation.x = y_error;
  msg.pose.orientation.y = th_error;
  msg.pose.orientation.z = new_error;
  msg.pose.orientation.w = new_error_rate;

  msg.pose.position.x    = steer;
  msg.pose.position.y    = steer;
  msg.pose.position.z    = steer;
  error_status_pub_.publish(msg);
}

void MessageToRos::MPCCostPublish(const float &x, const float &y, const float &z, const float &w)
{
  geometry_msgs::QuaternionStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.quaternion.x = x;
  msg.quaternion.y = y;
  msg.quaternion.z = z;
  msg.quaternion.w = w;

  mpc_cost_pub_.publish(msg);
}

void MessageToRos::VisualTargetPoint(const float &x, const float &y, const float &yaw)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";

  marker.header.stamp    = ros::Time::now();
  marker.ns              = "target_pose";
  marker.type            = visualization_msgs::Marker::ARROW;
  marker.action          = visualization_msgs::Marker::ADD;

  marker.pose.position.x  = x;
  marker.pose.position.y  = y;
  marker.pose.position.z  = 0.0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.lifetime = ros::Duration(0.0);
  target_pose_pub_.publish(marker);
}

void MessageToRos::VisualMpcPredictPoint(const std::vector<Eigen::Vector3f> &point)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";

  marker.header.stamp    = ros::Time::now();
  marker.ns              = "mpc_predict";
  marker.type            = visualization_msgs::Marker::SPHERE_LIST;
  marker.action          = visualization_msgs::Marker::ADD;

  for(int i=0;i<point.size();i++)
  {
    geometry_msgs::Point marker_point;
    marker_point.x = point.at(i)[0];
    marker_point.y = point.at(i)[1];
    marker_point.z = 0.0;

    marker.points.push_back(marker_point);
  }

  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;

  marker.lifetime = ros::Duration(0.0);
  prediction_point_pub_.publish(marker);
}

void MessageToRos::VisuallocalPath(const std::vector<nav_msgs::Path> &paths)
{
//  std::cout<<"局部路径1的备选条数为："<<paths.size()<<std::endl;
  for(int i=0;i<paths.size();i++)
  {
    nav_msgs::Path p = paths.at(i);

    visualization_msgs::Marker line_strip;
    line_strip.type            = visualization_msgs::Marker::LINE_STRIP;
    line_strip.header.stamp    = ros::Time::now();
    line_strip.header.frame_id = "map";
    line_strip.id              = i;
    line_strip.color.r         = 1.0;
    line_strip.color.g         = 0.0;
    line_strip.color.b         = 0.0;
    line_strip.color.a         = 1.0;
    line_strip.ns              = "multiple nav path";
    line_strip.action          = visualization_msgs::Marker::ADD;
    line_strip.scale.x         = 0.05;
    line_strip.scale.y         = 0.05;
    line_strip.scale.z         = 0.05;
    for(int j=0;j<p.poses.size();j++)
    {
      geometry_msgs::Point p1;
      p1.x = p.poses.at(j).pose.position.x;
      p1.y = p.poses.at(j).pose.position.y;
      p1.z = p.poses.at(j).pose.position.z;

      line_strip.points.push_back(p1);
    }
    if(i==0)
    {
      line_strip.color.r         = 1.0;
      line_strip.color.g         = 1.0;
      line_strip.color.b         = 0.0;
      line_strip.color.a         = 1.0;
      line_strip.scale.x         = 0.15;
      line_strip.scale.y         = 0.15;
      line_strip.scale.z         = 0.15;
    }
//    std::cout<<"i="<<i<<std::endl;
    line_strip.lifetime = ros::Duration(2.0);
    spline_pub_.publish(line_strip);
  }
}

void MessageToRos::VisuallocalPath(const std::vector<geometry_msgs::Point> &points)
{
  visualization_msgs::Marker line_strip;
  line_strip.type            = visualization_msgs::Marker::LINE_STRIP;
  line_strip.header.stamp    = ros::Time::now();
  line_strip.header.frame_id = "map";
  line_strip.id              = 1;
  line_strip.color.r         = 0.0;
  line_strip.color.g         = 0.0;
  line_strip.color.b         = 1.0;
  line_strip.color.a         = 1.0;
  line_strip.ns              = "ref local path";
  line_strip.action          = visualization_msgs::Marker::ADD;
  line_strip.scale.x         = 0.1;
  line_strip.scale.y         = 0.1;
  line_strip.scale.z         = 0.1;
  for(int j=0;j<points.size();j++)
  {
    geometry_msgs::Point p1;
    p1.x = points.at(j).x;
    p1.y = points.at(j).y;
    p1.z = 0.0;
    line_strip.points.push_back(p1);
  }
  line_strip.lifetime = ros::Duration();
  spline_pub_.publish(line_strip);
}

void MessageToRos::DebugPublish(const float &x, const float &y, const float &z, const float &w)
{
  geometry_msgs::QuaternionStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.quaternion.x = x;
  msg.quaternion.y = y;
  msg.quaternion.z = z;
  msg.quaternion.w = w;

  some_variable_pub_.publish(msg);
}

void MessageToRos::VisualFootPoint(const float &x, const float &y, const float &yaw)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";

  marker.header.stamp    = ros::Time::now();
  marker.ns              = "foot_print";
  marker.type            = visualization_msgs::Marker::ARROW;
  marker.action          = visualization_msgs::Marker::ADD;

  marker.pose.position.x  = x;
  marker.pose.position.y  = y;
  marker.pose.position.z  = 0.0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.lifetime = ros::Duration(0.0);
  foot_point_pub_.publish(marker);
}

}
