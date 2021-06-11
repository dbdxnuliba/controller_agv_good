#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  printf("\nmap_to_odom running ... \n\n");
  ros::init(argc, argv, "map_to_odom");
  ros::NodeHandle n;

  ros::Rate r(20);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"map", "odom"));
    //stage ros simulate
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
            ros::Time::now(),"map", "base_pose_ground_truth"));
#if 1
	broadcaster.sendTransform(
		  tf::StampedTransform(
		    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
		    ros::Time::now(),"odom", "base_link"));
#endif


    r.sleep();
  }
}

