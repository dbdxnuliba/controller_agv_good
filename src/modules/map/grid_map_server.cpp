#include "grid_map_server.h"
#include <stdlib.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <exception>
#include <string>
#include <set>
//linux
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/common.h"
#include "common/print.h"
#include "common/pnm_image.h"
#include "common/json.hpp"
#include "common/time_keeper.h"
#include "common/periodic_task.h"
#include "grid_map_data.h"
#include "grid_map.h"
#include "local_grid_map.h"
#include "contour.h"
#include "image_loader.h"
//#include "nng.h"

namespace bz_robot
{


GridMapServer::GridMapServer()
{
    std::map<std::string, std::string> nng_url;

    //init_nng_pub_server(nng_file);

//    mp_redis_puber_global_map = new RedisPuber(REDIS_IP, REDIS_PORT, MSG_CHANNEL_ID_MAP_UPDATE_GLOBAL_MAP);
//    mp_redis_puber_local_map = new RedisPuber(REDIS_IP, REDIS_PORT, MSG_CHANNEL_ID_MAP_UPDATE_LOCAL_MAP);
//    mp_redis_puber_robot_foot_print = new RedisPuber(REDIS_IP, REDIS_PORT, MSG_CHANNEL_ID_MAP_UPDATE_ROBOT_FOOT_PRINT);
//    mp_redis_getter_odom = new RedisGetter(REDIS_IP, REDIS_PORT, MSG_VALUE_ID_NAV_ODOM);

//    m_global_map_mtx.unlock();
//    m_local_map_mtx.unlock();
    m_is_imcoming_map_received = false;

//    m_lidar_sub = nh.subscribe("/lslidar_point_cloud", 1, &GridMapServer::on_lidar_data_update, this);

    mp_global_grid_map = std::make_shared<GridMap>();
    mp_local_grid_map = std::make_shared<LocalGridMap>();

    mp_global_grid_map->set_obstacles_cost(100);
    mp_local_grid_map->set_obstacles_cost(100);

    mp_global_grid_map->enable_inflaction(true);

    mp_local_grid_map->enable_inflaction(true);
    mp_local_grid_map->enable_obstacle_map(true);


    m_global_map_update_ms = 1000;
    m_local_map_update_ms = 50;
//    if(import_config(config_file))
//    {
//        std::vector<std::thread*> thread_list;
//        thread_list.push_back(new std::thread(&GridMapServer::update_global_map, this));
//        thread_list.push_back(new std::thread(&GridMapServer::update_local_map, this));
//        thread_list.push_back(new std::thread(&GridMapServer::deal_redis_sub_msgs, this));
//        for(auto it = thread_list.begin(); it != thread_list.end(); ++it)
//        {
//            (*it)->detach();
//        }
//    }
}

GridMapServer::~GridMapServer()
{

}

bool GridMapServer::import_config(const char *file)
{
    RECORD_TIME();
    try
    {
        m_is_imcoming_map_received = false;
        std::ifstream i(file);

        if(i)
        {
            nlohmann::json j;
            i >> j;
            std::string map_file = j["MAP_FILE"];
            float resoution = j["RESOLUTION"];
            float origin_x = j["ORIGIN_X"];
            float origin_y = j["ORIGIN_Y"];
            bool negate = j["NEGATE"];
            float occupied_thresh = j["OCCUPIED_THRESH"];
            float free_thresh = j["FREE_THRESH"];
            //get robot footprint
            std::vector<VectorX2<float>> contour;
            std::vector<std::vector<float> > foot_print = j["ROBOT_FOOT_PRINT"];
            for(int i = 0; i < foot_print.size(); i++)
            {
                contour.emplace_back(VectorX2<float>(foot_print[i][0],foot_print[i][1]));
            }
            mp_global_grid_map->set_robot_contour(contour);
            mp_local_grid_map->set_robot_contour(contour);

            m_incoming_map_data.origin_x = origin_x;
            m_incoming_map_data.origin_y = origin_y;
            load_map_from_file(&m_incoming_map_data, map_file.c_str(), resoution, negate, occupied_thresh, free_thresh);

            on_receive_incoming_map(m_incoming_map_data);

            {
                RECORD_TIME("global map set up finished");
                mp_global_grid_map->set_resolution(j["GLOBAL_MAP"]["RESOLUTION"]);
                m_global_map_update_ms = j["GLOBAL_MAP"]["UPDATE_TIME_MS"];
                float inflation_radius = j["GLOBAL_MAP"]["INFLATION_RADIUS"];
                float inflation_scaling_factor = j["GLOBAL_MAP"]["INFLATION_SCALING_FACTOR"];
                mp_global_grid_map->set_inflation_radius(inflation_radius, inflation_scaling_factor);
                mp_global_grid_map->enable_inflaction(j["GLOBAL_MAP"]["ENABLE_INFLATION_LAYER"]);
                mp_global_grid_map->enable_obstacle_map(j["GLOBAL_MAP"]["ENABLE_OBSTACLE_LAYER"]);
                mp_global_grid_map->set_obstacles_map_over_time(j["GLOBAL_MAP"]["OBSTACLE_OVER_TIME_MS"]);
            }

            {
                RECORD_TIME("local map set up finished");
                mp_local_grid_map->set_resolution(j["LOCAL_MAP"]["RESOLUTION"]);
                m_local_map_update_ms = j["LOCAL_MAP"]["UPDATE_TIME_MS"];
                float x_size = j["LOCAL_MAP"]["ROLLING_WINDOW_SIZE"][0];
                float y_size = j["LOCAL_MAP"]["ROLLING_WINDOW_SIZE"][1];
                mp_local_grid_map->set_rolling_window_size(x_size, y_size);
                float inflation_radius = j["LOCAL_MAP"]["INFLATION_RADIUS"];
                float inflation_scaling_factor = j["LOCAL_MAP"]["INFLATION_SCALING_FACTOR"];
                mp_local_grid_map->set_inflation_radius(inflation_radius, inflation_scaling_factor);
                mp_local_grid_map->enable_inflaction(j["LOCAL_MAP"]["ENABLE_INFLATION_LAYER"]);
                mp_local_grid_map->enable_obstacle_map(j["LOCAL_MAP"]["ENABLE_OBSTACLE_LAYER"]);
                mp_local_grid_map->set_obstacles_map_over_time(j["LOCAL_MAP"]["OBSTACLE_OVER_TIME_MS"]);
            }

            m_is_imcoming_map_received = true;
            PRINT_INFO("import map file success: {}", file);
            return true;
        }
        else
        {
            PRINT_ERROR("can't read config files from: {}\n", file);
        }
    }
    catch(std::exception& e )
    {
        PRINT_ERROR("exception: {}\n", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("un expexted occured\n");
    }
    return false;
}

std::shared_ptr<GridMap> GridMapServer::global_grid_map()
{
    return mp_global_grid_map;
}

std::shared_ptr<GridMap> GridMapServer::local_grid_map()
{
    return mp_local_grid_map;
}


void GridMapServer::on_receive_incoming_map(const GridMapData &new_map)
{
    {
        RECORD_TIME("recv global map");
        //m_global_map_mtx.lock();
        mp_global_grid_map->set_incoming_map_data(new_map);
        //m_global_map_mtx.unlock();
    }
    //progress local map
    {
        RECORD_TIME("recv local map");
        //m_local_map_mtx.lock();
        mp_local_grid_map->set_incoming_map_data(new_map);
        //m_local_map_mtx.unlock();
    }

    PRINT_INFO("finish progress map\n");
}

bool GridMapServer::update_global_map()
{
    //while (1)
    {
        //PERIODIC_TASK(m_global_map_update_ms);
        if (m_is_imcoming_map_received)
        {
//            Msg<Pose<FLOAT_T>> msg_odom = m_msg_odom;
//            mp_global_grid_map->set_robot_pose(msg_odom.data.position.x, msg_odom.data.position.y,
//                                              msg_odom.data.heading_angle);

            if(!mp_global_grid_map->is_static())
            {
                mp_global_grid_map->update_map();
            }
        }
    }
}

bool GridMapServer::update_local_map(const Msg<Pose<FLOAT_T>> &msg_odom)
{
    //while (1)
    {
        //PERIODIC_TASK(m_local_map_update_ms);
        //auto start = std::chrono::steady_clock::now();
        if(m_is_imcoming_map_received)
        {
            //RECORD_TIME();
//            std::lock_guard<std::mutex> temp_lock(m_local_map_mtx);
//            m_odom_mtx.lock();
//            mp_local_grid_map->set_robot_pose(m_odom_x, m_odom_y, m_odom_heading_angle);
//            m_odom_mtx.unlock();
            //Msg<Pose<float>> msg_odom;
            //Msg<Pose<FLOAT_T>> msg_odom = m_msg_odom;
            //mp_redis_getter_odom->get(&msg_odom);
            mp_local_grid_map->set_robot_pose(msg_odom.data.position.x, msg_odom.data.position.y,
                                              msg_odom.data.heading_angle);
            mp_local_grid_map->update_map();
        }
    }
}



//void GridMapServer::__on_sub_laser3d(const Msg<std::vector<VectorX3<float>>> &msg_point_cloud)
//{
//    //Msg<std::vector<VectorX3<float>>> msg_point_cloud;
//    Msg<Pose<float>> msg_odom = m_msg_odom;
//    //ia(str, len, &msg_point_cloud);

//    float world_x_size = 0;
//    float world_y_size = 0;
//    mp_local_grid_map->world_size(&world_x_size, &world_y_size);
//    const float min_z = -0.4;
//    const float max_z =  0.1;
////    m_odom_mtx.lock();
//    //mp_redis_getter_odom->get(&msg_odom);
//    const float odom_x = msg_odom.data.position.x;
//    const float odom_y = msg_odom.data.position.y;
//    const float heading_angle = -msg_odom.data.heading_angle;
////    m_odom_mtx.unlock();
//    const float cos_angle = cos(heading_angle);
//    const float sin_angle = sin(heading_angle);
//    std::vector<std::pair<float, float>> obstacles;
//    for(int i = 0; i < msg_point_cloud.data.size(); ++i)
//    {
//        /*
//                     * 这个地方可能有问题
//                     * 1 这个坐标值是障碍物相对激光雷达的位置，但是现在直接转成了在地图上面的坐标，是不是应该加上机器人的位置？
//                     * 2 现在默认激光雷达的x轴和y轴和地图的x轴，y轴重合，才能直接这么转换，缺乏坐标转换
//                     */
//        if(msg_point_cloud.data[i].z > min_z && msg_point_cloud.data[i].z < max_z)
//        {
//            float x = msg_point_cloud.data[i].x;
//            float y = msg_point_cloud.data[i].y;
//            float rotate_x = x * cos_angle + y * sin_angle;
//            float rotate_y = -x * sin_angle + y * cos_angle;

//            if(fabs(rotate_x*2) < world_x_size && fabs(rotate_y*2) < world_y_size)
//            {
//                x = odom_x + rotate_x;
//                y = odom_y + rotate_y;
//                obstacles.emplace_back(std::move(std::pair<float, float>(x, y)));
//            }
//        }
//    }
//    if(obstacles.size() > 0)
//    {
//        mp_global_grid_map->insert_obstacles_map(msg_point_cloud.time_stamp_us, obstacles);
//        mp_local_grid_map->insert_obstacles_map(msg_point_cloud.time_stamp_us, obstacles);
//    }
//}

//void GridMapServer::__on_sub_laser2d(const Msg<Laser2DData> &msg_laser2d)
//{
//    //Msg<LaserData> msg_laser2d;
//    Msg<Pose<float>> msg_odom = m_msg_odom;
//    //ia(str, len, &msg_laser2d);

//    float world_x_size = 0;
//    float world_y_size = 0;
//    mp_local_grid_map->world_size(&world_x_size, &world_y_size);
////    const float min_z = -0.4;
////    const float max_z =  0.1;
//    //    m_odom_mtx.lock();
//    //mp_redis_getter_odom->get(&msg_odom);
//    const float odom_x = msg_odom.data.position.x;
//    const float odom_y = msg_odom.data.position.y;
//    const float heading_angle = -msg_odom.data.heading_angle;
//    //    m_odom_mtx.unlock();
//    const float cos_angle = cos(heading_angle);
//    const float sin_angle = sin(heading_angle);
//    std::vector<std::pair<float, float>> obstacles;
//    const int64_t time_stamp = msg_laser2d.time_stamp_us;
//    const Laser2DData &laser_data = msg_laser2d.data;
//    for(int i = 0; i < laser_data.distance_list.size(); ++i)
//    {
//        float distance = laser_data.distance_list[i];
//        if(distance != 0)
//        {
//            float x = distance * cos(-laser_data.angle_offset.x + laser_data.start_angle + i * laser_data.angle_step);
//            float y = distance * sin(-laser_data.angle_offset.x + laser_data.start_angle + i * laser_data.angle_step);
////            if(laser_data.is_mirror)
////            {
////                y = -y;
////            }
//            x += laser_data.pose_offset.x;
//            y += laser_data.pose_offset.y;

//            float rotate_x = x * cos_angle + y * sin_angle;
//            float rotate_y = -x * sin_angle + y * cos_angle;

//            if(fabs(rotate_x*2) < world_x_size && fabs(rotate_y*2) < world_x_size)
//            {
//                x = odom_x + rotate_x;
//                y = odom_y + rotate_y;
//                obstacles.emplace_back(std::move(std::pair<float, float>(x, y)));
//            }
//        }
//    }
//    if(obstacles.size() > 0)
//    {
//        mp_global_grid_map->insert_obstacles_map(time_stamp, obstacles);
//        mp_local_grid_map->insert_obstacles_map(time_stamp, obstacles);
//    }
//}


//void GridMapServer::on_sub_laser3d(const char* str, const size_t& len, void *p_grid_map_server)
//{
//    Msg<std::vector<VectorX3<float>>> msg_point_cloud;
//    ia(str, len, &msg_point_cloud);
//    return ((GridMapServer*)p_grid_map_server)->__on_sub_laser3d(msg_point_cloud);
//}

//void GridMapServer::on_sub_laser2d(const char* str, const size_t& len, void *p_grid_map_server)
//{
//    Msg<LaserData> msg_laser2d;
//    ia(str, len, &msg_laser2d);
//    return ((GridMapServer*)p_grid_map_server)->__on_sub_laser2d(msg_laser2d);
//}



//void GridMapServer::on_sub_laser3d2(const void *p_msg, void *p_grid_map_server)
//{
//    const Msg<std::vector<VectorX3<float>>> &msg_point_cloud = *((Msg<std::vector<VectorX3<float>>>*)p_msg);
//    return ((GridMapServer*)p_grid_map_server)->__on_sub_laser3d(msg_point_cloud);
//}

//void GridMapServer::on_sub_laser2d2(const void *p_msg, void *p_grid_map_server)
//{
//    const Msg<LaserData> &msg_laser2d = *((Msg<LaserData>*)p_msg);
//    return ((GridMapServer*)p_grid_map_server)->__on_sub_laser2d(msg_laser2d);
//}

//void GridMapServer::deal_redis_sub_msgs()
//{
//#if 0
//    mp_redis_suber_laser3d = new RedisSuber(REDIS_IP, REDIS_PORT, MSG_CHANNEL_ID_ROBOT_SENSORS_LASER_POINT_CLOUD);
//    mp_redis_suber_laser2d = new RedisSuber(REDIS_IP, REDIS_PORT, MSG_CHANNEL_ID_ROBOT_SENSORS_LASER2D);

//    struct event_base* p_base = RedisSuber::build_listen_group();
//    mp_redis_suber_laser3d->sub(p_base, (GridMapServer::on_sub_laser3d), this);
//    mp_redis_suber_laser2d->sub(p_base, (GridMapServer::on_sub_laser2d), this);

//    RedisSuber::listen_loop(p_base);
//#else
//    mp_redis_suber_laser3d2 = new RedisSuber2<std::vector<VectorX3<float>>>(REDIS_IP, REDIS_PORT, MSG_CHANNEL_ID_ROBOT_SENSORS_LASER_POINT_CLOUD);
//    mp_redis_suber_laser2d2 = new RedisSuber2<LaserData>(REDIS_IP, REDIS_PORT, MSG_CHANNEL_ID_ROBOT_SENSORS_LASER2D);

//    struct event_base* p_base = build_listen_group();
//    mp_redis_suber_laser3d2->sub(p_base, (GridMapServer::on_sub_laser3d2), this);
//    mp_redis_suber_laser2d2->sub(p_base, (GridMapServer::on_sub_laser2d2), this);

//    listen_loop(p_base);
//#endif
//}



//bool bz_robot::GridMapServer::set_odom(const Msg<Pose<FLOAT_T>> &odom)
//{
//    m_msg_odom = odom;
//}

bool bz_robot::GridMapServer::set_laser2d_data(const Msg<LaserData> &msg_laser2d)
{
    //PRINT_DEBUG("{}", msg_laser2d.data.name);
    //const Laser2DData &laser_data = msg_laser2d.data;
//    float world_x_size = 0;
//    float world_y_size = 0;
//    mp_local_grid_map->world_size(&world_x_size, &world_y_size);

    const Msg<Pose<FLOAT_T>> &msg_odom = msg_laser2d.data.msg_location;
    const float odom_x = msg_odom.data.position.x;
    const float odom_y = msg_odom.data.position.y;
    const float heading_angle = -msg_odom.data.heading_angle;
    //    m_odom_mtx.unlock();
    const float cos_angle = cos(heading_angle);
    const float sin_angle = sin(heading_angle);
    GridMap::Obstacle obstacles;
    obstacles.time_stamp_us = msg_laser2d.time_stamp_us;
    obstacles.name = msg_laser2d.data.name;


    for(int i = 0; i < msg_laser2d.data.points.size(); ++i)
    {
        const FLOAT_T x = msg_laser2d.data.points[i].x;
        const FLOAT_T y = msg_laser2d.data.points[i].y;
        FLOAT_T rotate_x = x * cos_angle + y * sin_angle;
        FLOAT_T rotate_y = -x * sin_angle + y * cos_angle;
        obstacles.data.emplace_back(VectorX2<FLOAT_T>(odom_x + rotate_x, odom_y + rotate_y));
    }

    if(obstacles.data.size() > 0)
    {
        mp_global_grid_map->insert_obstacles(obstacles);
        mp_local_grid_map->insert_obstacles(obstacles);
    }
    return true;
}

bool GridMapServer::set_laser3d_data(const Msg<LaserData> &msg_laser3d)
{
    float world_x_size = 0;
    float world_y_size = 0;
    mp_local_grid_map->world_size(&world_x_size, &world_y_size);
    const float min_z = -0.4;
    const float max_z =  0.1;
    const float odom_x = msg_laser3d.data.msg_location.data.position.x;
    const float odom_y = msg_laser3d.data.msg_location.data.position.y;
    const float heading_angle = -msg_laser3d.data.msg_location.data.heading_angle;

    const float cos_angle = cos(heading_angle);
    const float sin_angle = sin(heading_angle);
    GridMap::Obstacle obstacles;
    obstacles.time_stamp_us = msg_laser3d.time_stamp_us;
    obstacles.name = msg_laser3d.data.name;
    for(int i = 0; i < msg_laser3d.data.points.size(); ++i)
    {

        if(msg_laser3d.data.points[i].z > min_z && msg_laser3d.data.points[i].z < max_z)
        {
            float x = msg_laser3d.data.points[i].x;
            float y = msg_laser3d.data.points[i].y;
            float rotate_x = x * cos_angle + y * sin_angle;
            float rotate_y = -x * sin_angle + y * cos_angle;

            if(fabs(rotate_x*2) < world_x_size && fabs(rotate_y*2) < world_y_size)
            {
                x = odom_x + rotate_x;
                y = odom_y + rotate_y;
                obstacles.data.emplace_back(std::move(VectorX2<FLOAT_T>(x, y)));
            }
        }
    }
    if(obstacles.data.size() > 0)
    {
        mp_global_grid_map->insert_obstacles(obstacles);
        mp_local_grid_map->insert_obstacles(obstacles);
    }

    return true;
}

}
