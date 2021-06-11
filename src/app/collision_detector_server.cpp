#include "map/grid_map_server.h"
#include "common/print.h"
#include "common/msg_id.h"
#include "third_libs/rest_rpc/include/rest_rpc.hpp"
#include "modules/collision_detector/collision_detector.h"
#include "modules/map/map_base.h"
#include "modules/map/grid_map_interface.h"


namespace bz_robot
{
    static std::shared_ptr<CollisionDetector> P_COLLISION_DETECTOR;
    static std::shared_ptr<rest_rpc::rpc_service::rpc_server> P_SERVER;

    static bool import_config(rpc_conn conn, const std::string config_file)
    {
        return P_COLLISION_DETECTOR->import_config(config_file.c_str());
    }

    static RetMsg<ControlData> calc_safety_velocity(rpc_conn conn, 
    	const Msg<GridMapData>& msg_map_data, 
    	const Msg<ControlData>& msg_control_data, 
        const Msg<Pose<FLOAT_T>>& msg_pose_robot)
    {
        static RetMsg<ControlData> ret_msg;
        std::shared_ptr<MapBase> p_map = std::make_shared<GridMapInterface>(msg_map_data.data);
        ret_msg.msg.data = P_COLLISION_DETECTOR->calc_safety_velocity(p_map, msg_control_data.data, msg_pose_robot.data);
        ret_msg.return_status = RET_SUCCESS;
        return ret_msg;
    }

    static bool is_collision(rpc_conn conn)
    {
        return P_COLLISION_DETECTOR->is_collision();
    }
}

int main()
{
    using namespace bz_robot;
    init_log("collision_detector_server");
    P_COLLISION_DETECTOR = std::make_shared<CollisionDetector>();
    rpc_server server(bz_robot::MSG_ID_SERVER_COLLISION_DETECTOR, 1, 0, 0);
    //bz_robot::P_GRID_MAP_SERVER = std::make_shared<bz_robot::GridMapServer>();

    server.register_handler("import_config", bz_robot::import_config);
    server.register_handler("calc_safety_velocity", bz_robot::calc_safety_velocity);
    server.register_handler("is_collision", bz_robot::is_collision);

    server.run();
}
