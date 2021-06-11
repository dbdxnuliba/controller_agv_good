#pragma once

#include <stdint.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>


#include "geometry.h"
#include "grid_map_data.h"
#include "common/data_types.h"
//#include "modules/robot/sensors/laser_data.h"
//#include "modules/msg/msg.h"

namespace bz_robot
{

class GridMap;
class LocalGridMap;

//template<typename T> class NanoMsg;
//class RedisPuber;

class GridMapServer
{
public:
    GridMapServer();
    ~GridMapServer();
    bool import_config(const char *file);
    std::shared_ptr<GridMap> global_grid_map();
    std::shared_ptr<GridMap> local_grid_map();

    //bool set_odom(const Msg<Pose<FLOAT_T> > &odom);
    bool set_laser2d_data(const Msg<LaserData> &msg_laser2d);
    bool set_laser3d_data(const Msg<LaserData> &msg_laser3d);

    bool update_global_map();
    bool update_local_map(const Msg<Pose<FLOAT_T> > &msg_odom);
private:
    void on_receive_incoming_map(const GridMapData& new_map);
//    void on_odom_update();
////    void sub_nng_msg();
//    void update_global_map();
//    void update_local_map();
//    //void init_nng_pub_server(const char *nng_file);
    //void __on_sub_laser3d(const Msg<std::vector<VectorX3<float>>> &msg_point_cloud);
    //void __on_sub_laser2d(const Msg<Laser2DData> &msg_laser2d);
//    static void on_sub_laser3d(const char* str, const size_t& len, void *p_grid_map_server);
//    static void on_sub_laser2d(const char* str, const size_t& len, void *p_grid_map_server);

//    static void on_sub_laser3d2(const void* p_msg, void *p_grid_map_server);
//    static void on_sub_laser2d2(const void* p_msg, void *p_grid_map_server);

//    void deal_redis_sub_msgs();
private:
    std::shared_ptr<GridMap> mp_global_grid_map;
    std::shared_ptr<LocalGridMap> mp_local_grid_map;


    std::atomic<bool> m_is_imcoming_map_received;

    int m_global_map_update_ms;
    int m_local_map_update_ms;

    GridMapData m_incoming_map_data;

    //Msg<Pose<FLOAT_T>> m_msg_odom;
    Laser2DData m_laser_2d_data;
    Laser3DData m_laser_3d_data;

//    int m_nng_pub_global_map;
//    int m_nng_pub_local_map;
//    int m_nng_pub_robot_foot_print;

//    int m_nng_sub_odom;
//    int m_nng_sub_laser_point_cloud;
//    int m_nng_sub_robot_laser2d;
//    int m_nng_sub_robot_sim_laser2d;

//    NanoMsg<Pose<float>> *mp_pose;
//    NanoMsg<std::vector<VectorX3<float>>> *mp_lidar_cloud;
//    NanoMsg<LaserData> *mp_laser_data;
//    NanoMsg<LaserData> *mp_sim_laser_data;
//    std::mutex m_mtx_odom;
//    std::mutex m_mtx_point_cloud;
//    std::mutex m_mtx_laser_data_list;
//    std::mutex m_mtx_sim_laser_data_list;

//    RedisPuber* mp_redis_puber_global_map;
//    RedisPuber* mp_redis_puber_local_map;
//    RedisPuber* mp_redis_puber_robot_foot_print;

//    RedisGetter* mp_redis_getter_odom;

//    RedisSuber* mp_redis_suber_laser3d;
//    RedisSuber* mp_redis_suber_laser2d;

//    RedisSuber2<std::vector<VectorX3<float>>> *mp_redis_suber_laser3d2;
//    RedisSuber2<LaserData> *mp_redis_suber_laser2d2;

};
}
