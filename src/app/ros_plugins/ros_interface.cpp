#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <thread>
#include <string>
#include <mutex>
#include <event2/event.h>
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
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include "common/common.h"
#include "common/json.hpp"
#include "common/periodic_task.h"
#include "common/print.h"
#include "common/time_keeper.h"
#include "modules/map/grid_map_server.h"
#include "common/data_types.h"
#include "common/msg_id.h"
#include "common/msg_id.h"

namespace bz_robot
{

static ros::Publisher ROS_PUB_GLOBAL_MAP;
static ros::Publisher ROS_PUB_LOCAL_MAP;
static ros::Publisher ROS_PUB_ROBOT_FOOT_PRINT;
static ros::Publisher ROS_PUB_GLOBAL_PLAN;
static ros::Publisher ROS_PUB_LOCAL_PLAN;
static ros::Publisher ROS_PUB_VELOCITY;
static ros::Publisher ROS_PUB_REAL_SENSE;


static Msg<Pose<FLOAT_T>> MSG_POSE_LOCATION;
static Msg<Pose<FLOAT_T>> MSG_POSE_START;
static Msg<Pose<FLOAT_T>> MSG_POSE_GOAL;
//static std::shared_ptr<boost::asio::ip::tcp::socket> P_SOCKET_TCP;
boost::asio::ip::tcp::socket *P_SOCKET_TCP;
std::shared_ptr<rest_rpc::rpc_client> P_CLIENT_MAP;
std::shared_ptr<rest_rpc::rpc_client> P_CLIENT_GLOBAL_PLANNER;
std::shared_ptr<rest_rpc::rpc_client> P_CLIENT_LOCAL_PLANNER;
std::shared_ptr<rest_rpc::rpc_client> P_CLIENT_LOCALIZATION;
std::shared_ptr<rest_rpc::rpc_client> P_CLIENT_ROBOT;
std::shared_ptr<rest_rpc::rpc_client> P_CLIENT_GLOBAL_SMOOTHER;
std::shared_ptr<rest_rpc::rpc_client> P_CLIENT_LOCAL_SMOOTHER;
static std::recursive_mutex MTX;

static inline void pack_msg(std::string* p_data)
{
    const size_t len = p_data->size();
    p_data->insert(0, "$#");
    p_data->insert(2, std::to_string(len).c_str());
    p_data->insert(2+std::to_string(len).size(), "##");
    p_data->insert(p_data->size(), "$~");
}

static void send_msg(std::string * msg)
{
#if 0
    boost::asio::io_service ios;
    boost::system::error_code error;
    boost::asio::ip::tcp::socket socket(ios);
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string("127.0.0.1"), 7273);
    socket.connect(endpoint, error);
    if(error)
    {
        PRINT_ERROR("init tcp client failed, error: {}", boost::system::system_error(error).what());
        return;
    }
    pack_msg(msg);
    PRINT_DEBUG("send goal msg:{}", *msg);
    socket.write_some(boost::asio::buffer(*msg));
#endif
    pack_msg(msg);
    PRINT_DEBUG("send msg:{}", *msg);
    P_SOCKET_TCP->write_some(boost::asio::buffer(*msg));
}

static void on_receive_map(const GridMapData &grid_map_data, ros::Publisher *p_pub)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
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
    std::lock_guard<std::recursive_mutex> lock(MTX);
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

static void _on_nng_receive_foot_print(const std::vector<VectorX2<float>> &contour_list, ros::Publisher *p_pub)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
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

static void _on_nng_receive_velocity(const ControlData &control_param, ros::Publisher *p_pub)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = control_param.velocity;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = control_param.steer_angle;
    p_pub->publish(cmd_vel);
}


static void on_init_pose_set(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr p_pose)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
    //    Pose<float> pose_start;
    Msg<Pose<float>> msg;
    msg.time_stamp_us = time_stamp_us();
    msg.data.position.x = p_pose->pose.pose.position.x;
    msg.data.position.y = p_pose->pose.pose.position.y;
    const float x = p_pose->pose.pose.orientation.x;
    const float y = p_pose->pose.pose.orientation.y;
    const float z = p_pose->pose.pose.orientation.z;
    const float w = p_pose->pose.pose.orientation.w;
    msg.data.heading_angle = quaternions_to_yaw(x, y, z, w);
    PRINT_INFO("receive start pose: {:.1f}, {:.1f}, {:.1f}", msg.data.position.x,
               msg.data.position.y, msg.data.heading_angle * 180 * M_1_PI);
    //send_data(ID_GROUP[NN_ID::ID_POSE_START], pose_start);
    P_CLIENT_LOCALIZATION->call_without_timeout("set_location", msg);
    //P_REDIS_PUB_POSE_START->pub(msg);
    MSG_POSE_START = std::move(msg);
    nlohmann::json j_out;
    j_out["#CMD#"] = "UmLocalize";
    j_out["#GAP#"] = -1;
    j_out["goal"] = "";
    j_out["poseTh"] = round(radian_to_degree(msg.data.heading_angle));
    j_out["poseX"] = round(msg.data.position.x * 1000);
    j_out["poseY"] = round(msg.data.position.y * 1000);
    j_out["target"] = "pose";
    std::string json_str  = j_out.dump();
    send_msg(&json_str);
//    pack_msg(&json_str);
//    P_SOCKET_TCP->write_some(boost::asio::buffer(json_str));
}

static void on_goal_pose_set(const geometry_msgs::PoseStamped::ConstPtr p_pose)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
    Msg<Pose<float>> msg;
    Pose<float> &pose_goal = msg.data;
    pose_goal.position.x = p_pose->pose.position.x;
    pose_goal.position.y = p_pose->pose.position.y;
    const float x = p_pose->pose.orientation.x;
    const float y = p_pose->pose.orientation.y;
    const float z = p_pose->pose.orientation.z;
    const float w = p_pose->pose.orientation.w;
    pose_goal.heading_angle = quaternions_to_yaw(x, y, z, w);

    PRINT_INFO("receive goal pose: {:.1f}, {:.1f}, {:.1f}", pose_goal.position.x,
               pose_goal.position.y, pose_goal.heading_angle * 180 * M_1_PI);
    //send_data(ID_GROUP[NN_ID::ID_POSE_GOAL], pose_goal);
    //P_REDIS_PUB_POSE_GOAL->pub(msg);
    MSG_POSE_GOAL = std::move(msg);

    nlohmann::json j_out;
    j_out["#CMD#"] = "UmGoto";
    j_out["#GAP#"] = -1;
    j_out["goal"] = "none";
    j_out["poseTh"] = round(radian_to_degree(pose_goal.heading_angle));
    j_out["poseX"] = round(pose_goal.position.x * 1000);
    j_out["poseY"] = round(pose_goal.position.y * 1000);
    j_out["target"] = "pose";
    std::string json_str  = j_out.dump();
    send_msg(&json_str);
//    pack_msg(&json_str);
//    PRINT_DEBUG("send goal msg:{}", json_str);
//    P_SOCKET_TCP->write_some(boost::asio::buffer(json_str));
}

static void on_odom_update(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
    //PRINT_DEBUG("odom :{:.3f}", time_stamp_us() * 0.001 * 0.001);
//    PRINT_DEBUG("START");
    Pose<float> pose_robot;
    pose_robot.position.x = odom_msg->pose.pose.position.x;
    pose_robot.position.y = odom_msg->pose.pose.position.y;
    pose_robot.heading_angle = quaternions_to_yaw(odom_msg->pose.pose.orientation.x,
                                                  odom_msg->pose.pose.orientation.y,
                                                  odom_msg->pose.pose.orientation.z,
                                                  odom_msg->pose.pose.orientation.w);

    MSG_POSE_LOCATION.time_stamp_us = time_stamp_us();
    MSG_POSE_LOCATION.data = pose_robot;

    P_CLIENT_LOCALIZATION->call_without_timeout("set_location", MSG_POSE_LOCATION);


    nlohmann::json j_out;
    j_out["#CMD#"] = "UmLocalize";
    j_out["#GAP#"] = -1;
    j_out["goal"] = "";
    j_out["poseTh"] = round(radian_to_degree(pose_robot.heading_angle));
    j_out["poseX"] = round(pose_robot.position.x * 1000);
    j_out["poseY"] = round(pose_robot.position.y * 1000);
    j_out["target"] = "pose";
    std::string json_str  = j_out.dump();
    //send_msg(&json_str);
//    PRINT_DEBUG("END");
//    pack_msg(&json_str);
//    P_SOCKET_TCP->write_some(boost::asio::buffer(json_str));
}


void on_lidar_data_update(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    {
        //PRINT_DEBUG("START");
        pcl::fromROSMsg(*input, cloud);
    }
    Msg<Laser3DData> msg;
    //Msg<std::vector<VectorX3<float>>> msg;
    msg.time_stamp_us = time_stamp_us();
    msg.data.name = "LSLIDAR";
    msg.data.msg_location = MSG_POSE_LOCATION;
    const uint32_t len = cloud.points.size();
    msg.data.points.resize(len);
    VectorX3<float> data;

    for(int i = 0; i < len; ++i)
    {
        data.x = cloud.points[i].x;
        data.y = cloud.points[i].y;
        data.z = cloud.points[i].z;
        msg.data.points[i] = std::move(data);
    }
    P_CLIENT_ROBOT->call("set_laser3d_data", msg);
}


void on_simulator_stage_laser_scan(const boost::shared_ptr<const sensor_msgs::LaserScan> &input)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
    //PRINT_DEBUG("START");
    Msg<Laser2DData> msg;
    msg.time_stamp_us = time_stamp_us();
    Laser2DData &laser_data = msg.data;
    laser_data.name = "ROS_STAGE_SIM";
    laser_data.msg_location = MSG_POSE_LOCATION;
    laser_data.pose_offset = VectorX3<FLOAT_T>();
    laser_data.angle_offset = VectorX3<FLOAT_T>();

    laser_data.angle_step = input->angle_increment;
    laser_data.start_angle = input->angle_min;
    //laser_data.distance_list = input->ranges;
    const int len = input->ranges.size();
    laser_data.distance_list.resize(len);
    for(int i = 0; i < len; ++i)
    {
        if(input->ranges[i] >= input->range_max)
        {
            laser_data.distance_list[i] = 0;
        }
        else
        {
            //printf("[%d] %2.2f  ", input->ranges[i]);
            laser_data.distance_list[i] = input->ranges[i];
        }
    }
    //printf("\n");
    P_CLIENT_ROBOT->call("set_laser2d_data", msg);
    //PRINT_DEBUG("END");
    //P_REDIS_PUB_LASER2D->pub(msg);
    //send_data(ID_GROUP[NN_ID::ID_SIM_LASER2D], laser_data);
}

void on_ros_recv_reconfig_msg(const std_msgs::String &msg)
{
    nlohmann::json j_out;
    j_out["#CMD#"] = "BZRobotReconfig";
    j_out["#GAP#"] = -1;
    j_out["config_file"] = msg.data.c_str();
    std::string json_str  = j_out.dump();
    send_msg(&json_str);
}
void on_ros_recv_realsense_scan(const boost::shared_ptr<const sensor_msgs::LaserScan> &input)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
    Msg<Laser2DData> msg;
    msg.time_stamp_us = time_stamp_us();
    Laser2DData &laser_data = msg.data;
    laser_data.pose_offset.x = 0.6;
    laser_data.angle_step = input->angle_increment;
    laser_data.start_angle = input->angle_min;
    //laser_data.distance_list = input->ranges;
    //PRINT_DEBUG("angle min = {}", radian_to_degree(input->angle_min));
    const int len = input->ranges.size();
    laser_data.distance_list.resize(len);
    for(int i = 0; i < len; ++i)
    {
        if(isnan(input->ranges[i]))
        {
            laser_data.distance_list[i] = 0;
        }
        else
        {
            laser_data.distance_list[i] = input->ranges[i];
        }
        //        if(input->ranges[i] >= input->range_max)
        //        {
        //            laser_data.distance_list[i] = 0;
        //        }
        //        else
        //        {
        //            laser_data.distance_list[i] = input->ranges[i];
        //        }
    }
    //P_REDIS_PUB_LASER2D->pub(msg);
    //send_data(ID_GROUP[NN_ID::ID_SIM_LASER2D], laser_data);
}

static void on_sub_foot_print2(const void *msg, void *arg)
{
    _on_nng_receive_foot_print(((Msg<FootPrintData>*)msg)->data, & ROS_PUB_ROBOT_FOOT_PRINT);
}

static void on_sub_velocity2(const void *msg, void *arg)
{
    _on_nng_receive_velocity(((Msg<ControlData>*)msg)->data, & ROS_PUB_VELOCITY);
}


static void publish_map_boundles(ros::Publisher *p_pub)
{
    while(1)
    {
        geometry_msgs::PolygonStamped polygon_stamped;
        polygon_stamped.header.stamp = ros::Time::now();
        polygon_stamped.header.frame_id = "map";
        polygon_stamped.polygon.points.resize(4);

        polygon_stamped.polygon.points[0].x = 3.20607;
        polygon_stamped.polygon.points[0].y = -20.2124;
        polygon_stamped.polygon.points[1].x = -21.8977;
        polygon_stamped.polygon.points[1].y = 101.158;
        polygon_stamped.polygon.points[2].x = 179.001;
        polygon_stamped.polygon.points[2].y = 130.774;
        polygon_stamped.polygon.points[3].x = 204.389;
        polygon_stamped.polygon.points[3].y = 27.2871;

        p_pub->publish(polygon_stamped);

    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}


//static bool init_client()
//{
//    boost::asio::io_service ios;
//    boost::system::error_code error;
//    P_SOCKET_TCP = std::make_shared<boost::asio::ip::tcp::socket>(ios);
//    //boost::asio::ip::tcp::socket socket(ios);
//    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string("127.0.0.1"), 7273);
//    P_SOCKET_TCP->connect(endpoint, error);
//    if (error)
//    {
//        PRINT_ERROR("init tcp client failed, error: {}", boost::system::system_error(error).what());
//        return false;
//    }
//    PRINT_INFO("connect {}:{} success", "127.0.0.1", 7273);
//    return true;
//}

static void event_update_global_map(int fd, short events, void *p_event_params)
{
//    PRINT_DEBUG("START");
    EventParam * p_param = (EventParam *) p_event_params;
    evtimer_add(p_param->ev, &p_param->tv);
    bool r = P_CLIENT_MAP->connect(10);
    if (!r)
    {
        PRINT_ERROR("connect map server failed");
        return;
    }
   const GridMapData &grid_map_data =  P_CLIENT_MAP->call<RetMsg<GridMapData>>("global_map").msg.data;
   on_receive_map(grid_map_data, &ROS_PUB_GLOBAL_MAP);
//   PRINT_DEBUG("END");
}

static void event_update_local_map(int fd, short events, void *p_event_params)
{
//    PRINT_DEBUG("START");
    EventParam * p_param = (EventParam *) p_event_params;
    evtimer_add(p_param->ev, &p_param->tv);
    bool r = P_CLIENT_MAP->connect(0);
    if (!r)
    {
        PRINT_ERROR("connect map server failed");
        return;
    }
    const GridMapData &grid_map_data =  P_CLIENT_MAP->call<RetMsg<GridMapData>>("local_map").msg.data;
    on_receive_map(grid_map_data, &ROS_PUB_LOCAL_MAP);
//    PRINT_DEBUG("END");
}


static void event_update_global_path(int fd, short events, void *p_event_params)
{
    EventParam * p_param = (EventParam *) p_event_params;
    evtimer_add(p_param->ev, &p_param->tv);
    bool r = P_CLIENT_GLOBAL_SMOOTHER->connect(10);
    if (!r)
    {
        PRINT_ERROR("connect global planner server failed");
        return;
    }
    const PathData &path_data =  P_CLIENT_GLOBAL_SMOOTHER->call<RetMsg<PathData>>("smoothed_path").msg.data;
    on_receive_plan(path_data, &ROS_PUB_GLOBAL_PLAN);
}


static void event_update_local_path(int fd, short events, void *p_event_params)
{
    EventParam * p_param = (EventParam *) p_event_params;
    evtimer_add(p_param->ev, &p_param->tv);
    bool r = P_CLIENT_LOCAL_SMOOTHER->connect(10);
    if (!r)
    {
        PRINT_ERROR("connect local planner server failed");
        return;
    }
    const PathData &path_data =  P_CLIENT_LOCAL_SMOOTHER->call<RetMsg<PathData>>("smoothed_path").msg.data;
    on_receive_plan(path_data, &ROS_PUB_LOCAL_PLAN);
}

static void background_tasks()
{
    struct event_base* p_event_base = event_base_new();

    EventParam * param_global_map = (EventParam*)malloc(sizeof(EventParam));
    param_global_map->ev = evtimer_new(p_event_base, event_update_global_map, param_global_map);
    param_global_map->tv.tv_sec = 5;
    param_global_map->tv.tv_usec = 0;
    param_global_map->p_args = nullptr;
    evtimer_add(param_global_map->ev, &param_global_map->tv);

    EventParam * param_local_map = (EventParam*)malloc(sizeof(EventParam));
    param_local_map->ev = evtimer_new(p_event_base, event_update_local_map, param_local_map);
    param_local_map->tv.tv_sec = 0;
    param_local_map->tv.tv_usec = 200 * 1000;
    param_local_map->p_args = nullptr;
    evtimer_add(param_local_map->ev, &param_local_map->tv);

    EventParam * param_global_path = (EventParam*)malloc(sizeof(EventParam));
    param_global_path->ev = evtimer_new(p_event_base, event_update_global_path, param_global_path);
    param_global_path->tv.tv_sec = 1;
    param_global_path->tv.tv_usec = 0;
    param_global_path->p_args = nullptr;
    evtimer_add(param_global_path->ev, &param_global_path->tv);

    EventParam * param_local_path = (EventParam*)malloc(sizeof(EventParam));
    param_local_path->ev = evtimer_new(p_event_base, event_update_local_path, param_local_path);
    param_local_path->tv.tv_sec = 0;
    param_local_path->tv.tv_usec = 100 * 1000;
    param_local_path->p_args = nullptr;
    evtimer_add(param_local_path->ev, &param_local_path->tv);

    int err = event_base_dispatch(p_event_base);
    PRINT_ERROR("error = {}", err);
    event_base_free(p_event_base);
}

static void sub_tracker_velocity()
{
    static rest_rpc::rpc_client client;
    client.enable_auto_reconnect();
    client.enable_auto_heartbeat();
    bool r = client.connect("127.0.0.1", MSG_ID_TASK_NAVIGATION);
    if (!r) {
        return;
    }

    client.subscribe("tracker_velocity", [](string_view data) {
        msgpack_codec codec;
        Msg<ControlData> msg = codec.unpack<Msg<ControlData>>(data.data(), data.size());
        //PRINT_DEBUG("recv tracker velocity: {}, {}", msg.data.velocity, radian_to_degree(msg.data.steer_angle));
        _on_nng_receive_velocity(msg.data, &ROS_PUB_VELOCITY);
    });
}


}
int main(int argc, char **argv)
{
    //std::this_thread::sleep_for(std::chrono::seconds(10));
    using namespace bz_robot;
    try
    {
        init_log("ros_interface");
        boost::asio::io_service ios;
        boost::system::error_code error;
        //P_SOCKET_TCP = std::make_shared<boost::asio::ip::tcp::socket>(ios);
        P_SOCKET_TCP = new boost::asio::ip::tcp::socket(ios);
        //boost::asio::ip::tcp::socket socket(ios);
        boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string("127.0.0.1"), 7273);
        //socket.connect(endpoint, error);
        int try_times = 0;
        bool is_connect_server_success = false;
        while(try_times < 3)
        {
            ++try_times;
            P_SOCKET_TCP->connect(endpoint, error);
            if(error)
            {
                PRINT_WARN("init tcp client failed, error: {}", boost::system::system_error(error).what());
                PRINT_WARN("wait 2 seconds.");
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            else
            {
                is_connect_server_success = true;
                break;
            }
        }
        if(!is_connect_server_success)
        {
            PRINT_ERROR("connect server error !!!");
            abort();
        }


        //init_client();
        P_CLIENT_MAP = std::make_shared<rest_rpc::rpc_client>("127.0.0.1", MSG_ID_SERVER_MAP);
        P_CLIENT_GLOBAL_PLANNER = std::make_shared<rest_rpc::rpc_client>("127.0.0.1", MSG_ID_SERVER_GLOBAL_PLAN);
        P_CLIENT_LOCAL_PLANNER = std::make_shared<rest_rpc::rpc_client>("127.0.0.1", MSG_ID_SERVER_LOCAL_PLAN);
        P_CLIENT_LOCALIZATION = std::make_shared<rest_rpc::rpc_client>("127.0.0.1", MSG_ID_SERVER_LOCALIZATION);
        P_CLIENT_ROBOT = std::make_shared<rest_rpc::rpc_client>("127.0.0.1", MSG_ID_SERVER_ROBOT);
        P_CLIENT_GLOBAL_SMOOTHER = std::make_shared<rest_rpc::rpc_client>("127.0.0.1", MSG_ID_SERVER_GLOBAL_PATH_SMOOTH);
        P_CLIENT_LOCAL_SMOOTHER = std::make_shared<rest_rpc::rpc_client>("127.0.0.1", MSG_ID_SERVER_LOCAL_PATH_SMOOTH);


        P_CLIENT_MAP->connect(0);
        P_CLIENT_GLOBAL_PLANNER->connect(0);
        P_CLIENT_LOCAL_PLANNER->connect(0);
        P_CLIENT_LOCALIZATION->connect(0);
        P_CLIENT_ROBOT->connect(0);
        P_CLIENT_GLOBAL_SMOOTHER->connect(0);
        P_CLIENT_LOCAL_SMOOTHER->connect(0);

        ros::init(argc, argv, "ros_interface");
        ros::NodeHandle nh;
        //pub
        bz_robot::ROS_PUB_GLOBAL_MAP = nh.advertise<nav_msgs::OccupancyGrid>("BZRobotGlobalGridMap", 1);
        bz_robot::ROS_PUB_LOCAL_MAP = nh.advertise<nav_msgs::OccupancyGrid>("BZRobotLocalGridMap", 1);
        bz_robot::ROS_PUB_ROBOT_FOOT_PRINT = nh.advertise<geometry_msgs::PolygonStamped>("BZRobotRobotFootPrint", 1);
        bz_robot::ROS_PUB_GLOBAL_PLAN = nh.advertise<nav_msgs::Path>("bz_robot_global_plan", 1);
        bz_robot::ROS_PUB_LOCAL_PLAN = nh.advertise<nav_msgs::Path>("bz_robot_local_plan", 1);
        bz_robot::ROS_PUB_VELOCITY = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        bz_robot::ROS_PUB_REAL_SENSE = nh.advertise<sensor_msgs::PointCloud2>("test_real_sense", 1);
        //sub
        ros::Subscriber ros_sub_init_pose =
            nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, bz_robot::on_init_pose_set);
        ros::Subscriber ros_sub_goal_pose =
            nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, bz_robot::on_goal_pose_set);
        ros::Subscriber ros_sub_odom = nh.subscribe("/odom", 1, bz_robot::on_odom_update);
        //base_pose_ground_truth
        // ros::Subscriber odom_sub = nh.subscribe("base_pose_ground_truth", 1, on_odom_update);
        ros::Subscriber ros_sub_point_clout = nh.subscribe("/lslidar_point_cloud", 1, bz_robot::on_lidar_data_update);
        ros::Subscriber ros_sub_stage_laser_scan = nh.subscribe("/base_scan", 1, bz_robot::on_simulator_stage_laser_scan);
        ros::Subscriber ros_sub_intel_realsense = nh.subscribe("/scan_filtered", 1, bz_robot::on_ros_recv_realsense_scan);
        ros::Subscriber ros_sub_reconfig= nh.subscribe("/BZRobotReconfig", 1, bz_robot::on_ros_recv_reconfig_msg);


        std::vector<std::thread*> thread_list;
        //thread_list.push_back(new std::thread(bz_robot::on_recv_redis_msg));
        //thread_list.push_back(new std::thread(bz_robot::publish_map_boundles, &bz_robot::ROS_PUB_ROBOT_FOOT_PRINT));
        thread_list.push_back(new std::thread(background_tasks));
        for(auto it = thread_list.begin(); it != thread_list.end(); ++it)
        {
            (*it)->detach();
        }
        sub_tracker_velocity();
        ros::spin();
    }
    catch(std::exception& e )
    {
        PRINT_ERROR("{}\n", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("unexpexted error occured\n");
    }
    return 0;
}
