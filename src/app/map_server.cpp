//#include <rest_rpc.hpp>
//#include <fstream>
//#include "map/grip_map_server.h"
#include "modules/map/grid_map.h"
#include "modules/map/grid_map_server.h"
#include "map/grid_map_server.h"
#include "common/print.h"
#include "third_libs/rest_rpc/include/rest_rpc.hpp"
#include "common/msg_id.h"

namespace bz_robot
{
    static std::shared_ptr<GridMapServer> P_GRID_MAP_SERVER;
    static std::shared_ptr<rest_rpc::rpc_service::rpc_server> P_SERVER;

    static bool import_map_config(rpc_conn conn, const std::string config_file)
    {
        return P_GRID_MAP_SERVER->import_config(config_file.c_str());
    }

//    static bool set_odom(rpc_conn conn, const Msg<Pose<FLOAT_T>> &odom)
//    {
//        return P_GRID_MAP_SERVER->set_odom(odom);
//    }

    static void update_global_map(rpc_conn conn)
    {
        P_GRID_MAP_SERVER->update_global_map();
    }

    static void update_local_map(rpc_conn conn, const Msg<Pose<FLOAT_T>> &msg_odom)
    {
        P_GRID_MAP_SERVER->update_local_map(msg_odom);
    }

    static RetMsg<GridMapData> global_map(rpc_conn conn)
    {
        static RetMsg<GridMapData> ret_msg;
        P_GRID_MAP_SERVER->global_grid_map()->map_data(&ret_msg.msg.data);
        ret_msg.return_status = RET_SUCCESS;
        return ret_msg;
    }

    static RetMsg<GridMapData> local_map(rpc_conn conn)
    {
        static RetMsg<GridMapData> ret_msg;

        P_GRID_MAP_SERVER->local_grid_map()->map_data(&ret_msg.msg.data);
        ret_msg.return_status = RET_SUCCESS;
        return ret_msg;
    }

    static bool set_laser2d_data(rpc_conn conn, const Msg<Laser2DData> &msg_laser2d)
    {
        return P_GRID_MAP_SERVER->set_laser2d_data(msg_laser2d);
    }

    static bool set_laser3d_data(rpc_conn conn, const Msg<Laser3DData> &msg_laser3d)
    {
        return P_GRID_MAP_SERVER->set_laser3d_data(msg_laser3d);
    }
}

int main()
{
    init_log("map_server");
    rpc_server server(bz_robot::MSG_ID_SERVER_MAP, 2, 0, 0);
    bz_robot::P_GRID_MAP_SERVER = std::make_shared<bz_robot::GridMapServer>();

    server.register_handler("import_config", bz_robot::import_map_config);
    //server.register_handler("set_odom", bz_robot::set_odom);
    server.register_handler("update_global_map", bz_robot::update_global_map);
    server.register_handler("update_local_map", bz_robot::update_local_map);
    server.register_handler("global_map", bz_robot::global_map);
    server.register_handler("local_map", bz_robot::local_map);
    server.register_handler("set_laser2d_data", bz_robot::set_laser2d_data);
    server.register_handler("set_laser3d_data", bz_robot::set_laser3d_data);

    server.run();
}
