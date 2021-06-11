#include "common/data_types.h"
#include "common/json.hpp"
#include "common/print.h"
#include "third_libs/rest_rpc/include/rest_rpc.hpp"
#include "common/msg_id.h"
#include "robot/body_control.h"
#include "robot/sensor_manager.h"
#include "modules/collision_detector/collision_detector.h"
#include "map/grid_map_data.h"

namespace bz_robot
{
std::string BodyControl::server_url = "http://192.168.1.110:20000/RPC2";
static std::shared_ptr<rest_rpc::rpc_client> P_COLLISION_DETECTOR_CLIENT;
static std::shared_ptr<SensorManager> P_LASER_MANAGER = std::make_shared<SensorManager>();
static std::recursive_mutex MTX;
static Msg<ControlData> MSG_CONTROL_DATA;
static bool FLAG_SIMULATE = false;
static bool FLAG_SAFETY = true;

static ReturnStatus import_config(rpc_conn conn, const std::string &config_file)
{
    ReturnStatus return_status = RET_ERROR;
    try
    {
        PRINT_INFO("read config: {}", config_file);
        std::ifstream i(config_file);
        if(i)
        {
            nlohmann::json j;
            i >> j;
            FLAG_SIMULATE = j["SIMULATE_FLAG"];
            std::string url = j["BODY_CONTROL_SERVER_URL"];
            BodyControl::server_url = url;
            const std::string &sensor_file = j["SENSOR_LIST"];
            P_LASER_MANAGER->import_config(sensor_file);
            return_status = RET_SUCCESS;
        }
        else
        {
            PRINT_ERROR("can't read config files from: {}\n", config_file);
        }
    }
    catch(std::exception& e )
    {
        PRINT_ERROR("{}\n", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("un expexted occured\n");
    }
    return return_status;
}

static ReturnStatus set_control_data(rpc_conn conn, const Msg<GridMapData>& msg_map_data,
                                     const Msg<ControlData>& msg_control_data,
                                     const Msg<Pose<FLOAT_T>>& msg_pose_robot)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
    MSG_CONTROL_DATA = msg_control_data;
    ReturnStatus ret_status = RET_SUCCESS;
    if(FLAG_SAFETY)
    {
        if(P_COLLISION_DETECTOR_CLIENT->connect())
        {
            RetMsg<ControlData> ret_msg = P_COLLISION_DETECTOR_CLIENT->call<RetMsg<ControlData>>("calc_safety_velocity",
                                                                                                 msg_map_data,
                                                                                                 msg_control_data,
                                                                                                 msg_pose_robot);
            if(ret_msg.return_status == RET_SUCCESS)
            {
                MSG_CONTROL_DATA = ret_msg.msg;
            }
        }
    }
    if(!FLAG_SIMULATE)
    {
        BodyControl::set_velocity_and_angle(MSG_CONTROL_DATA.data.velocity, MSG_CONTROL_DATA.data.steer_angle, 1);
    }
    return ret_status;
}

static RetMsg<ControlData> feedback_control_data(rpc_conn conn)
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
    static RetMsg<ControlData> ret_msg;
    ret_msg.return_status = RET_SUCCESS;
    ret_msg.msg = MSG_CONTROL_DATA;
    return ret_msg;
}

static ReturnStatus update_laser2d(rpc_conn conn, const std::string &name)
{
    return P_LASER_MANAGER->update_laser_2d(name);
}

static ReturnStatus set_laser2d_data(rpc_conn conn, const Msg<Laser2DData>& msg)
{
    ReturnStatus ret = RET_ERROR;
    bool result = P_LASER_MANAGER->set_laser2d_data(msg);
    if(result)
    {
        ret = RET_SUCCESS;
    }
    return ret;
}

static RetMsg<Laser2DData> laser2d_data(rpc_conn conn, const std::string &name)
{
    static RetMsg<Laser2DData> ret_msg;
    bool result = P_LASER_MANAGER->laser2d_data(name, &ret_msg.msg);
    if(result)
    {
        ret_msg.return_status = RET_SUCCESS;
    }
    else
    {
        ret_msg.return_status = RET_ERROR;
    }
    return ret_msg;
}

static ReturnStatus update_laser3d(rpc_conn conn, const std::string &name)
{
    return P_LASER_MANAGER->update_laser_3d(name);
}

static void set_laser3d_data(rpc_conn conn, const Msg<Laser3DData>& msg)
{
    P_LASER_MANAGER->set_laser3d_data(msg);
}

static RetMsg<Laser3DData> laser3d_data(rpc_conn conn, const std::string &name)
{
    static RetMsg<Laser3DData> ret_msg;
    bool result = P_LASER_MANAGER->laser3d_data(name, &ret_msg.msg);
    if(result)
    {
        ret_msg.return_status = RET_SUCCESS;
    }
    else
    {
        ret_msg.return_status = RET_ERROR;
    }
    return ret_msg;
}


}

int main()
{
    using namespace bz_robot;
    init_log("robot_server");
    //P_GRID_MAP_SERVER = std::make_shared<bz_robot::GridMapServer>();
    P_COLLISION_DETECTOR_CLIENT = std::make_shared<rest_rpc::rpc_client>("127.0.0.1", MSG_ID_SERVER_COLLISION_DETECTOR);
    rpc_server server(bz_robot::MSG_ID_SERVER_ROBOT, 2, 0, 0);
    server.register_handler("import_config", bz_robot::import_config);
    server.register_handler("set_control_data", bz_robot::set_control_data);
    server.register_handler("feedback_control_data", bz_robot::feedback_control_data);
    server.register_handler("update_laser2d", bz_robot::update_laser2d);
    server.register_handler("set_laser2d_data", bz_robot::set_laser2d_data);
    server.register_handler("laser2d_data", bz_robot::laser2d_data);
    server.register_handler("update_laser3d", bz_robot::update_laser3d);
    server.register_handler("set_laser3d_data", bz_robot::set_laser3d_data);
    server.register_handler("laser3d_data", bz_robot::laser3d_data);

    server.run();
}
