#pragma once

#include <mutex>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>

#include <laserscan_kinect/LaserscanKinectConfig.h>
#include <laserscan_kinect/laserscan_kinect.h>

namespace laserscan_kinect {

class LaserScanKinectNode {
 public:
  /**
   * @brief LaserScanKinectNode constructor.
   *
   * @param n  Node handler.
   * @param pnh Private node handler.
   */
  LaserScanKinectNode(ros::NodeHandle& n, ros::NodeHandle& pnh);
  ~LaserScanKinectNode();

private:
 
  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
 
  void connectCb(const ros::SingleSubscriberPublisher& pub);
  
  void disconnectCb(const ros::SingleSubscriberPublisher& pub);
 
  void reconfigureCb(laserscan_kinect::LaserscanKinectConfig &config, uint32_t level);

  /// Private node handler used to generate the transport hints in the connectCb.
  ros::NodeHandle pnh_;
  /// Subscribes to synchronized Image CameraInfo pairs.
  image_transport::ImageTransport it_;
  /// Subscriber for image_transport
  image_transport::CameraSubscriber sub_;
  /// Publisher for output LaserScan messages
  ros::Publisher pub_;
  /// Dynamic reconfigure server
  dynamic_reconfigure::Server<laserscan_kinect::LaserscanKinectConfig> srv_;
  /// Object which convert depth image to laserscan and store all parameters
  laserscan_kinect::LaserScanKinect converter_;
  /// Prevents the connectCb and disconnectCb from being called until everything is initialized.
  std::mutex connect_mutex_;
};

class LaserScanRealsenseNode {
 public:
  /**
   * @brief LaserScanKinectNode constructor.
   *
   * @param n  Node handler.
   * @param pnh Private node handler.
   */
  LaserScanRealsenseNode(ros::NodeHandle& n, ros::NodeHandle& pnh);
  ~LaserScanRealsenseNode();

private:
 
  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
 
  void connectCb(const ros::SingleSubscriberPublisher& pub);
  
  void disconnectCb(const ros::SingleSubscriberPublisher& pub);
 
  void reconfigureCb(laserscan_kinect::LaserscanKinectConfig &config, uint32_t level);

  /// Private node handler used to generate the transport hints in the connectCb.
  ros::NodeHandle pnh_;
  /// Subscribes to synchronized Image CameraInfo pairs.
  image_transport::ImageTransport it_;
  /// Subscriber for image_transport
  image_transport::CameraSubscriber sub_;
  /// Publisher for output LaserScan messages
  ros::Publisher pub_;
  /// Dynamic reconfigure server
  dynamic_reconfigure::Server<laserscan_kinect::LaserscanKinectConfig> srv_;
  /// Object which convert depth image to laserscan and store all parameters
  laserscan_kinect::LaserScanKinect converter_;
  /// Prevents the connectCb and disconnectCb from being called until everything is initialized.
  std::mutex connect_mutex_;
};







} // namespace laserscan_kinect