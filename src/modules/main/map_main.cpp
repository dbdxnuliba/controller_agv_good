#include "modules/map/grid_map.h"
#include "modules/map/grid_map_server.h"
#include "map/grid_map_server.h"
#include "common/print.h"
#include "common/common.h"
#include "common/thread_rpc.h"
#include "common/msg_id.h"
#include "modules/main/map_main.h"


namespace bz_robot
{
    static std::shared_ptr<GridMapServer> P_GRID_MAP_SERVER;
    //static std::shared_ptr<rest_rpc::rpc_service::rpc_server> P_SERVER;

    static bool import_map_config(const std::string config_file)
    {
        return P_GRID_MAP_SERVER->import_config(config_file.c_str());
    }

    static void update_global_map()
    {
        P_GRID_MAP_SERVER->update_global_map();
    }

    static void update_local_map(const Msg<Pose<FLOAT_T>> &msg_odom)
    {
        P_GRID_MAP_SERVER->update_local_map(msg_odom);
    }

    static RetMsg<GridMapData> global_map()
    {
        static RetMsg<GridMapData> ret_msg;
        P_GRID_MAP_SERVER->global_grid_map()->map_data(&ret_msg.msg.data);
        ret_msg.return_status = RET_SUCCESS;
        return ret_msg;
    }

    static RetMsg<GridMapData> local_map()
    {
        static RetMsg<GridMapData> ret_msg;
        P_GRID_MAP_SERVER->local_grid_map()->map_data(&ret_msg.msg.data);
        ret_msg.return_status = RET_SUCCESS;
        return ret_msg;
    }

    static bool set_laser2d_data(const Msg<LaserData> &msg_laser2d)
    {
        return P_GRID_MAP_SERVER->set_laser2d_data(msg_laser2d);
    }

    static bool set_laser3d_data(const Msg<LaserData> &msg_laser3d)
    {
        return P_GRID_MAP_SERVER->set_laser3d_data(msg_laser3d);
    }


void map_module_main(int argc, char **argv)
{
    //using namespace bz_robot;
    //init_log("map_server");
    PRINT_INFO("init");
    thread_rpc::Server server(MACRO_STR(MSG_ID_SERVER_MAP), 2);
    bz_robot::P_GRID_MAP_SERVER = std::make_shared<GridMapServer>();

    server.register_handler("import_config", import_map_config);
    //server.register_handler("set_odom", set_odom);
    server.register_handler("update_global_map", update_global_map, 1);
    server.register_handler("update_local_map", update_local_map);
    server.register_handler("global_map", global_map, 1);
    server.register_handler("local_map", local_map);
    server.register_handler("set_laser2d_data", set_laser2d_data);
    server.register_handler("set_laser3d_data", set_laser3d_data);

    server.run();
}
}
