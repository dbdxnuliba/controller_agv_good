#include "task_query_status.h"
#include "common/common.h"
#include "common/msg_id.h"
#include "common/thread_rpc.h"

#include "map/grid_map_interface.h"


namespace bz_robot
{

class TaskQueryStatusParams
{
public:
    std::shared_ptr<thread_rpc::Client> p_client_robot;
    std::shared_ptr<thread_rpc::Client> p_client_localization;
    std::shared_ptr<thread_rpc::Client> p_client_map;
    std::shared_ptr<thread_rpc::Client> p_client_collision_detector;
};

TaskQueryStatus::TaskQueryStatus()
{
    mp_params = std::make_shared<TaskQueryStatusParams>();
    mp_params->p_client_collision_detector = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_COLLISION_DETECTOR));
    mp_params->p_client_localization = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_LOCALIZATION));
    mp_params->p_client_map = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_MAP));
    mp_params->p_client_robot = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_ROBOT));

}

TaskQueryStatus::~TaskQueryStatus()
{

}

bool TaskQueryStatus::safe_drive_status() const
{
    return mp_params->p_client_collision_detector->call<bool>("enable_collision_detector");
}

std::string TaskQueryStatus::robot_name() const
{
    return std::string("bz_robot");
}

std::string TaskQueryStatus::map_name() const
{
    return std::string("bz_map");
}

GridMapData TaskQueryStatus::global_map() const
{
    return mp_params->p_client_map->call<RetMsg<GridMapData>>("global_map").msg.data;
}

PathData TaskQueryStatus::global_path() const
{
    return mp_params->p_client_robot->call<RetMsg<PathData>>("global_planner_path").msg.data;
}

PathData TaskQueryStatus::local_path() const
{
    return mp_params->p_client_robot->call<RetMsg<PathData>>("local_planner_path").msg.data;
}

int TaskQueryStatus::localization_status() const
{
    return 90;
}

bool TaskQueryStatus::motor_state() const
{
    return true;
}

RobotInfo TaskQueryStatus::robot_info() const
{
    RobotInfo robot_info;
    robot_info.pose = mp_params->p_client_localization->call<RetMsg<Pose<FLOAT_T>>>("location").msg.data;
    robot_info.control_data = mp_params->p_client_robot->call<RetMsg<ControlData>>("feedback_control_data").msg.data;
    robot_info.battery = 100;
    robot_info.mode = std::string("");
    robot_info.status = std::string("");
    return robot_info;
}


}
