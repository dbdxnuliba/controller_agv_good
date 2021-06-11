#include "task/task_map.h"
#include <fstream>
#include "common/msg_id.h"
#include "common/common.h"
#include "common/thread_rpc.h"
#include "common/periodic_task.h"
#include "common/print.h"
#include "common/json.hpp"
#include "common/time_keeper.h"
#include "common/timer/timer.h"


namespace bz_robot
{

class TaskMapParams
{
public:
    std::shared_ptr<thread_rpc::Client> p_client_map;
    std::shared_ptr<thread_rpc::Client> p_client_localization;
    uint32_t global_map_update_cycle_time_ms = 2000;
    uint32_t local_map_update_cycle_time_ms = 200;
};

TaskMap::TaskMap()
{
    mp_params = std::make_shared<TaskMapParams>();
    mp_params->p_client_map = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_MAP));
    mp_params->p_client_localization = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_LOCALIZATION));
}

TaskMap::~TaskMap()
{

}

bool TaskMap::import_config(const char *file_path)
{
    bool result = false;
    try
    {
        //std::ifstream i(file_path);
        //if(i)
        {
            //nlohmann::json j;
            //i >> j;
            //std::string map_config_file = j["MAP_CONFIG"];

            std::ifstream i_map_file(file_path);
            nlohmann::json j_map;
            i_map_file >> j_map;
            mp_params->global_map_update_cycle_time_ms = j_map["GLOBAL_MAP"]["UPDATE_TIME_MS"];
            mp_params->local_map_update_cycle_time_ms = j_map["LOCAL_MAP"]["UPDATE_TIME_MS"];
            //mp_params->p_client_map->call("import_config", file_path);

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


bool TaskMap::register_background_tasks()
{

    bz_robot::timer::Timer &t = timer::GetTimer::timer();
    t.add_task("update_global_map", mp_params->global_map_update_cycle_time_ms, &TaskMap::update_global_map, this, nullptr);
    t.add_task("update_local_map", mp_params->local_map_update_cycle_time_ms, &TaskMap::update_local_map, this, nullptr);
    return true;

}

inline const TaskMapParams * const TaskMap::params() const
{
    return mp_params.get();
}

void TaskMap::update_global_map(timer::TimerTaskItem *p_task, void *p_params)
{
    p_task->add_task(mp_params->global_map_update_cycle_time_ms, p_params);
    mp_params->p_client_map->call("update_global_map");
}

void TaskMap::update_local_map(timer::TimerTaskItem *p_task, void *p_params)
{
    p_task->add_task(mp_params->local_map_update_cycle_time_ms, p_params);
    const Msg<Pose<FLOAT_T>> msg_location = std::move((mp_params->p_client_localization->call<RetMsg<Pose<FLOAT_T>>>("location")).msg);
    mp_params->p_client_map->call("update_local_map", msg_location);
}

Msg<Pose<FLOAT_T> > TaskMap::location()
{
    return mp_params->p_client_localization->call<RetMsg<Pose<FLOAT_T>>>("location").msg;
}


}
