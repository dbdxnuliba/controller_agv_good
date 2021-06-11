#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <thread>
#include <string>
#include <mutex>

//ros header
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_types.h>
//#include <pcl/PCLPointCloud2.h>
//#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>
#include "plugins_ros_debug.h"
#include "common/common.h"
#include "common/thread_rpc.h"
#include "common/json.hpp"
#include "common/periodic_task.h"
#include "common/print.h"
#include "common/time_keeper.h"
#include "modules/map/grid_map_server.h"
#include "common/data_types.h"
#include "common/msg_id.h"
#include "common/timer/timer.h"

namespace bz_robot
{

static ros::NodeHandle* P_ROS_NH = nullptr;
static bool ENABLE_ROS_DEBUG = true;


static void on_receive_map(const GridMapData &grid_map_data, ros::Publisher *p_pub)
{
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();

    grid.header.frame_id = "map";
    grid.info.resolution = (float)grid_map_data.resolution;
    grid.info.width = grid_map_data.x_size;
    grid.info.height = grid_map_data.y_size;

    grid.info.origin.position.x = grid_map_data.origin_x;
    grid.info.origin.position.y = grid_map_data.origin_y;
    grid.info.origin.position.z = 0;
    grid.info.origin.orientation.w = 1;
    uint32_t len = grid_map_data.x_size *  grid_map_data.y_size;
    grid.data.assign(len, 0);
//    for(auto it = grid_map_data.obstacles_index_set.begin(); it != grid_map_data.obstacles_index_set.end(); ++it)
//    {
//        const uint64_t &index = *it;
//        grid.data[index] = 100;
//    }
    for(uint32_t i = 0; i < len; ++i)
    {
        uint32_t x = i % grid_map_data.x_size;
        uint32_t y = i / grid_map_data.x_size;
        grid.data[i] = grid_map_data.map[x][y];
    }
    p_pub->publish(grid);
}

static void on_receive_plan(const PathData &path, ros::Publisher *p_pub)
{
    ros::Time plan_time = ros::Time::now();
    std::vector<geometry_msgs::PoseStamped> plan;
    nav_msgs::Path gui_path;
    gui_path.header.frame_id = "map";
    gui_path.poses.resize(path.size());
    gui_path.header.stamp = plan_time;
    for(int i = 0; i < path.size(); ++i)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = plan_time;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = path[i].position.x;
        pose_stamped.pose.position.y = path[i].position.y;
        pose_stamped.pose.position.z = 0;
        float x = 0;
        float y = 0;
        float z = 0;
        float w = 0;
        yaw_to_quaternions(path[i].heading_angle, &x, &y, &z, &w);
        pose_stamped.pose.orientation.x = x;
        pose_stamped.pose.orientation.y = y;
        pose_stamped.pose.orientation.z = z;
        pose_stamped.pose.orientation.w = w;
        gui_path.poses[i] = pose_stamped;
    }
    p_pub->publish(gui_path);
}

static void on_receive_foot_print(const std::vector<VectorX2<float>> &contour_list, ros::Publisher *p_pub)
{
    geometry_msgs::PolygonStamped polygon_stamped;
    polygon_stamped.header.stamp = ros::Time::now();
    polygon_stamped.header.frame_id = "map";
    polygon_stamped.polygon.points.resize(contour_list.size());
    for(uint32_t i = 0; i < contour_list.size(); ++i)
    {
        polygon_stamped.polygon.points[i].x = contour_list[i].x;
        polygon_stamped.polygon.points[i].y = contour_list[i].y;
    }
    p_pub->publish(polygon_stamped);
}

static void on_receive_velocity(const ControlData &control_param, ros::Publisher *p_pub)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = control_param.velocity;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = control_param.steer_angle;
    p_pub->publish(cmd_vel);
}


void pub_path(const std::string name, const PathData path)
{
	if(ENABLE_ROS_DEBUG)
	{
		if(P_ROS_NH)
        {
			ros::Publisher pb = P_ROS_NH->advertise<nav_msgs::Path>(name, 1);
			ros::Time plan_time = ros::Time::now();
		    std::vector<geometry_msgs::PoseStamped> plan;
		    nav_msgs::Path gui_path;
		    gui_path.header.frame_id = "map";
		    gui_path.poses.resize(path.size());
		    gui_path.header.stamp = plan_time;
		    for(int i = 0; i < path.size(); ++i)
		    {
		        geometry_msgs::PoseStamped pose_stamped;
		        pose_stamped.header.stamp = plan_time;
		        pose_stamped.header.frame_id = "map";
		        pose_stamped.pose.position.x = path[i].position.x;
		        pose_stamped.pose.position.y = path[i].position.y;
		        pose_stamped.pose.position.z = 0;
		        float x = 0;
		        float y = 0;
		        float z = 0;
		        float w = 0;
		        yaw_to_quaternions(path[i].heading_angle, &x, &y, &z, &w);
		        pose_stamped.pose.orientation.x = x;
		        pose_stamped.pose.orientation.y = y;
		        pose_stamped.pose.orientation.z = z;
		        pose_stamped.pose.orientation.w = w;
		        gui_path.poses[i] = pose_stamped;
		    }
            pb.publish(gui_path);
		}
	}
}

void pub_path_list(const std::string name, const std::vector<PathData> path_list)
{
	if(ENABLE_ROS_DEBUG)
	{
		if(P_ROS_NH)
        {
            ros::Publisher pb = P_ROS_NH->advertise<nav_msgs::Path>(name, 1);
            ros::Time plan_time = ros::Time::now();
            std::vector<geometry_msgs::PoseStamped> plan;
            nav_msgs::Path gui_path;
            gui_path.header.frame_id = "map";

            for(int i = 0; i < path_list.size(); ++i)
            {
                const PathData& path = path_list[i];
                gui_path.poses.resize(path.size());
                gui_path.header.stamp = plan_time;
                for(int i = 0; i < path.size(); ++i)
                {
                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = plan_time;
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = path[i].position.x;
                    pose_stamped.pose.position.y = path[i].position.y;
                    pose_stamped.pose.position.z = 0;
                    float x = 0;
                    float y = 0;
                    float z = 0;
                    float w = 0;
                    yaw_to_quaternions(path[i].heading_angle, &x, &y, &z, &w);
                    pose_stamped.pose.orientation.x = x;
                    pose_stamped.pose.orientation.y = y;
                    pose_stamped.pose.orientation.z = z;
                    pose_stamped.pose.orientation.w = w;
                    gui_path.poses[i] = pose_stamped;
                }
                pb.publish(gui_path);
            }
        }
	}
}

void plugins_ros_debug_main(int argc, char **argv)
{
    try
    {
        
        P_ROS_NH = new ros::NodeHandle();
       	
       	thread_rpc::Server server(MACRO_STR(MSG_ID_PLUGINS_ROS_DEBUG));
        server.register_handler("ros_pub_path", pub_path);
        server.register_handler("ros_pub_path_list", pub_path_list);

        server.run();
    }
    catch(std::exception& e )
    {
        PRINT_ERROR("{}\n", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("unexpexted error occured\n");
    }
}

}
