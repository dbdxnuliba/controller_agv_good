#include "task/task_drive.h"
#include <fstream>
#include "common/msg_id.h"
#include "common/common.h"
#include "common/thread_rpc.h"
#include "common/periodic_task.h"
#include "common/print.h"
#include "common/json.hpp"
#include "common/time_keeper.h"
#include "common/timer/timer.h"
#include "modules/map/grid_map_interface.h"

namespace bz_robot
{

class TaskDriveParams
{
public:
    std::shared_ptr<thread_rpc::Client> p_client_robot;
    std::shared_ptr<thread_rpc::Client> p_client_localization;
    std::shared_ptr<thread_rpc::Client> p_client_map;
    uint32_t control_effective_time_ms;
    uint32_t monitor_time_ms;
    timer::TimerTaskItem *p_task;
    Msg<ControlData> msg_control_cmd;
    std::mutex mtx;
};

TaskDrive::TaskDrive()
{
    mp_params = std::make_shared<TaskDriveParams>();
    mp_params->control_effective_time_ms = 200;
    mp_params->monitor_time_ms = 30;
    mp_params->p_task = nullptr;
    mp_params->p_client_robot = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_ROBOT));
    mp_params->p_client_localization = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_LOCALIZATION));
    mp_params->p_client_map = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_MAP));
}

TaskDrive::~TaskDrive()
{

}

bool TaskDrive::import_config(const char *file_path)
{
    bool result = false;
    try
    {
        std::ifstream i(file_path);
        if(i)
        {
            nlohmann::json j;
            i >> j;
//            mp_params->global_map_update_cycle_time_ms = j["GLOBAL_MAP"]["UPDATE_TIME_MS"];
//            mp_params->local_map_update_cycle_time_ms = j["LOCAL_MAP"]["UPDATE_TIME_MS"];
            result = true;
        }

    }
    catch(std::exception& e )
    {
        PRINT_ERROR("exception: {}\n", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("unexpexted occured\n");
    }
    return result;

}

bool TaskDrive::set_cmd(const std::string &cmd)
{
    std::lock_guard<std::mutex> lock(mp_params->mtx);
    nlohmann::json j = nlohmann::json::parse(cmd);
    mp_params->msg_control_cmd.time_stamp_us = time_stamp_us();
    mp_params->msg_control_cmd.data.velocity = j["velocity"];
    mp_params->msg_control_cmd.data.steer_angle = j["rot"];

    return true;
}

RetMsg<std::string> TaskDrive::run()
{
    //mp_params->msg_control_cmd.time_stamp_us = time_stamp_us();
    add_monitor();
    RetMsg<std::string> ret_msg;
    RetMsg<GridMapData> ret_msg_map_data = mp_params->p_client_map->call<RetMsg<GridMapData>>("local_map");
    RetMsg<Pose<FLOAT_T>> ret_msg_pose = mp_params->p_client_localization->call<RetMsg<Pose<FLOAT_T>>>("location");
    std::lock_guard<std::mutex> lock(mp_params->mtx);
    if(ret_msg_map_data.return_status == RET_SUCCESS && ret_msg_pose.return_status == RET_SUCCESS)
    {
        //PRINT_DEBUG("velocity: {}, steer_ange: {}", mp_params->msg_control_cmd.data.velocity, radian_to_degree(mp_params->msg_control_cmd.data.steer_angle));
        mp_params->p_client_robot->call("set_control_data", ret_msg_map_data.msg, mp_params->msg_control_cmd, ret_msg_pose.msg);
    }
    return ret_msg;
}

void TaskDrive::timer_monitor_velocity(timer::TimerTaskItem *p_task, void *p_params)
{
    std::lock_guard<std::mutex> lock(mp_params->mtx);
    if(p_task != nullptr)
    {
        mp_params->p_task = p_task;
    }
    int64_t time_ms_interval = (time_stamp_us() - mp_params->msg_control_cmd.time_stamp_us) / 1000;
    if(time_ms_interval < mp_params->control_effective_time_ms)
    {
        if(p_task)
        {
            p_task->add_task(mp_params->monitor_time_ms, p_params);
        }
    }
    else
    {
        mp_params->p_client_robot->call("stop");
        PRINT_WARN("time_ms_interval = {}", time_ms_interval);
        PRINT_WARN("over time, stop robot\n");
        mp_params->p_task = nullptr;
    }

}


bool TaskDrive::register_background_tasks()
{
//    bz_robot::timer::Timer &t = timer::GetTimer::timer();
//    t.add_task(mp_params->control_effective_time_ms, &TaskDrive::timer_monitor_velocity, this, nullptr);
    return true;
}

void TaskDrive::add_monitor()
{
    std::lock_guard<std::mutex> lock(mp_params->mtx);
    if(mp_params->p_task == nullptr)
    {
        bz_robot::timer::Timer &t = timer::GetTimer::timer();
        t.add_task("timer_monitor_velocity", mp_params->monitor_time_ms, &TaskDrive::timer_monitor_velocity, this, nullptr);
    }
}


}
