#include "common/data_types.h"
#include "common/json.hpp"
#include "common/print.h"
#include "common/common.h"
#include "common/thread_rpc.h"
#include "common/time_keeper.h"
#include "common/msg_id.h"
#include "robot/body_control.h"
#include "robot/sensor_manager.h"
#include "modules/collision_detector/collision_detector.h"
#include "map/grid_map_data.h"
#include "modules/main/robot_main.h"

namespace bz_robot
{
//std::string BodyControl::server_url = "http://192.168.1.110:20000/RPC2";
static std::shared_ptr<thread_rpc::Client> P_COLLISION_DETECTOR_CLIENT;

static std::shared_ptr<SensorManager> P_LASER_MANAGER = std::make_shared<SensorManager>();
static std::recursive_mutex MTX;
static Msg<ControlData> MSG_CONTROL_DATA;
static bool FLAG_SIMULATE = false;
static bool FLAG_SAFETY = true;
static RetMsg<PathData> RET_MSG_GLOBAL_PLANNER_PATH;
static RetMsg<PathData> RET_MSG_LOCAL_PLANNER_PATH;



static ReturnStatus import_config(const std::string &config_file)
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

static ReturnStatus stop()
{
    std::lock_guard<std::recursive_mutex> lock(MTX);
    MSG_CONTROL_DATA.data.steer_angle = 0;
    MSG_CONTROL_DATA.data.velocity = 0;
    bool status = true;
    if(!FLAG_SIMULATE)
    {
        status = BodyControl::set_velocity_and_angle(MSG_CONTROL_DATA.data.velocity, MSG_CONTROL_DATA.data.steer_angle, 1);
    }
    ReturnStatus ret_status = (status? RET_SUCCESS: RET_FAILED);
    return ret_status;
}

static RetMsg<ControlData> feedback_control_data()
{
    //RECORD_TIME();
    std::lock_guard<std::recursive_mutex> lock(MTX);
    static RetMsg<ControlData> ret_msg;
    ret_msg.return_status = RET_SUCCESS;
    ret_msg.msg = MSG_CONTROL_DATA;
    if(FLAG_SIMULATE)
    {
        ret_msg.return_status = RET_SUCCESS;
        ret_msg.msg = MSG_CONTROL_DATA;
    }
    else
    {
        ret_msg.return_status = RET_SUCCESS;
        ret_msg.msg.data.velocity = BodyControl::feedback_velocity() * 0.001;
        ret_msg.msg.data.steer_angle = MSG_CONTROL_DATA.data.steer_angle;
        //ret_msg.msg.data.steer_angle = degree_to_radian(-40.0 * BodyControl::feedback_angle() / 42233.0);
    }
    //PRINT_INFO("v: {:.2f}, angle {:.1f}", ret_msg.msg.data.velocity, radian_to_degree(ret_msg.msg.data.steer_angle));
    return ret_msg;
}


static ReturnStatus set_control_data(const Msg<GridMapData>& msg_map_data,
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

static ReturnStatus update_laser2d(const std::string &name)
{
    return P_LASER_MANAGER->update_laser_2d(name);
}

static ReturnStatus set_laser2d_data(const Msg<Laser2DData>& msg)
{
    ReturnStatus ret = RET_ERROR;
    bool result = P_LASER_MANAGER->set_laser2d_data(msg);
    if(result)
    {
        ret = RET_SUCCESS;
    }
    return ret;
}

static RetMsg<LaserData> laser2d_data(const std::string &name)
{
    static RetMsg<LaserData> ret_msg;
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

static ReturnStatus update_laser3d(const std::string &name)
{
    return P_LASER_MANAGER->update_laser_3d(name);
}

static void set_laser3d_data(const Msg<Laser3DData>& msg)
{
    P_LASER_MANAGER->set_laser3d_data(msg);
}

static RetMsg<LaserData> laser3d_data(const std::string &name)
{
    static RetMsg<LaserData> ret_msg;
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

static void set_global_planner_path(const Msg<PathData>& msg)
{
    RET_MSG_GLOBAL_PLANNER_PATH.return_status = RET_SUCCESS;
    RET_MSG_GLOBAL_PLANNER_PATH.msg = msg;
}

static RetMsg<PathData> global_planner_path()
{
    return RET_MSG_GLOBAL_PLANNER_PATH;
}

static void set_local_planner_path(const Msg<PathData>& msg)
{
    RET_MSG_LOCAL_PLANNER_PATH.return_status = RET_SUCCESS;
    RET_MSG_LOCAL_PLANNER_PATH.msg = msg;
}

static RetMsg<PathData> local_planner_path()
{
    return RET_MSG_LOCAL_PLANNER_PATH;
}

void robot_module_main(int argc, char **argv)
{
    //using namespace bz_robot;
    PRINT_INFO("init");
    //init_log("robot_server");
    //P_GRID_MAP_SERVER = std::make_shared<bz_robot::GridMapServer>();
    P_COLLISION_DETECTOR_CLIENT = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_COLLISION_DETECTOR));
    thread_rpc::Server server(MACRO_STR(MSG_ID_SERVER_ROBOT));
    server.register_handler("import_config", bz_robot::import_config);
    server.register_handler("stop", bz_robot::stop);
    server.register_handler("set_control_data", bz_robot::set_control_data);
    server.register_handler("feedback_control_data", bz_robot::feedback_control_data);
    server.register_handler("update_laser2d", bz_robot::update_laser2d);
    server.register_handler("set_laser2d_data", bz_robot::set_laser2d_data);
    server.register_handler("laser2d_data", bz_robot::laser2d_data);
    server.register_handler("update_laser3d", bz_robot::update_laser3d);
    server.register_handler("set_laser3d_data", bz_robot::set_laser3d_data);
    server.register_handler("laser3d_data", bz_robot::laser3d_data);
    server.register_handler("set_global_planner_path", bz_robot::set_global_planner_path);
    server.register_handler("global_planner_path", bz_robot::global_planner_path);
    server.register_handler("set_local_planner_path", bz_robot::set_local_planner_path);
    server.register_handler("local_planner_path", bz_robot::local_planner_path);

    server.run();
}
}
