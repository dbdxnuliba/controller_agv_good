#include "map/grid_map_server.h"
#include "common/common.h"
#include "common/print.h"
#include "common/msg_id.h"
#include "common/thread_rpc.h"
#include "modules/collision_detector/collision_detector.h"
#include "modules/map/map_base.h"
#include "modules/map/grid_map_interface.h"
#include "modules/main/collision_detector_main.h"

namespace bz_robot
{
    static std::shared_ptr<CollisionDetector> P_COLLISION_DETECTOR;

    static bool import_config(const std::string& config_file)
    {
        return P_COLLISION_DETECTOR->import_config(config_file.c_str());
    }

    static RetMsg<ControlData> calc_safety_velocity(
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

    static bool is_collision()
    {
        return P_COLLISION_DETECTOR->is_collision();
    }

    static bool enable_collision_detector()
    {
        return P_COLLISION_DETECTOR->enable_collision_detector();
    }

    void collision_detector_module_main(int argc, char **argv)
    {
        using namespace bz_robot;
        PRINT_INFO("init");
        P_COLLISION_DETECTOR = std::make_shared<CollisionDetector>();
        thread_rpc::Server server(MACRO_STR(MSG_ID_SERVER_COLLISION_DETECTOR), 1);


        server.register_handler("import_config", bz_robot::import_config);
        server.register_handler("calc_safety_velocity", bz_robot::calc_safety_velocity);
        server.register_handler("is_collision", bz_robot::is_collision);
        server.register_handler("enable_collision_detector", bz_robot::enable_collision_detector);

        server.run();
    }
}
