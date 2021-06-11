#include "task/task_sensor.h"
#include <fstream>
#include "common/msg_id.h"
#include "common/common.h"
#include "common/thread_rpc.h"
#include "common/periodic_task.h"
#include "common/print.h"
#include "common/json.hpp"
#include "robot/sensor_base.h"
#include "common/time_keeper.h"
#include "common/timer/timer.h"

namespace bz_robot
{

//static void event_update_laser_2d(int fd, short events, void * p_event_params);
//static void event_update_laser_3d(int fd, short events, void * p_event_params);

typedef struct {
    TaskSensor *p_task;
    std::string name;
}TaskSensorEventParam;

class TaskSensorParams
{
public:
    std::shared_ptr<thread_rpc::Client> p_client_robot;
    std::shared_ptr<thread_rpc::Client> p_client_localization;
    std::shared_ptr<thread_rpc::Client> p_client_map;
//    uint32_t global_map_update_cycle_time_ms = 2000;
//    uint32_t local_map_update_cycle_time_ms = 200;
    std::vector<std::string> laser_2d_list;
    std::unordered_map<std::string, uint32_t> laser_2d_update_cycle_time_ms;
    std::vector<std::string> laser_3d_list;
    std::unordered_map<std::string, uint32_t> laser_3d_update_cycle_time_ms;

    std::vector<std::string* > p_task_params_laser_2d;
    std::vector<std::string* > p_task_params_laser_3d;
};

TaskSensor::TaskSensor()
{
    mp_params = std::make_shared<TaskSensorParams>();
    mp_params->laser_2d_list.clear();
    mp_params->p_client_robot = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_ROBOT));
    mp_params->p_client_localization = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_LOCALIZATION));
    mp_params->p_client_map = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_MAP));
    mp_params->p_client_robot->connect();
    mp_params->p_client_localization->connect();
    mp_params->p_client_map->connect();
}

TaskSensor::~TaskSensor()
{
    for(int i = 0; i < mp_params->p_task_params_laser_2d.size(); ++i)
    {
        delete mp_params->p_task_params_laser_2d[i];
    }
    for(int i = 0; i < mp_params->p_task_params_laser_3d.size(); ++i)
    {
        delete mp_params->p_task_params_laser_3d[i];
    }
}

bool TaskSensor::import_config(const char *file_path)
{
    RECORD_TIME();
    bool result = false;
    try
    {
        std::ifstream i(file_path);
        if(i)
        {
            nlohmann::json j;
            i >> j;
            //std::cout << j["SENSOR_LIST"] << "\n";
            const std::string &sensor_file = j["SENSOR_LIST"];
            nlohmann::json j_sensor;
            std::ifstream i_sensor(sensor_file);
            if(i_sensor)
            {
                i_sensor >> j_sensor;
                //std::cout << j_sensor["LASER_2D_LIST"] << "\n";
                const std::string laser_2d_config_file = j_sensor["LASER_2D_LIST"];
                const std::string laser_3d_config_file = j_sensor["LASER_3D_LIST"];
                std::ifstream i_laser_2d_file(laser_2d_config_file);
                if(i_laser_2d_file)
                {
                    nlohmann::json j_laser_2d;
                    i_laser_2d_file >> j_laser_2d;
                    //std::cout << j_laser_2d["LASER_2D"] << "\n";
                    std::vector<nlohmann::json> laser2d_config = j_laser_2d["LASER_2D"];
                    for(int i = 0; i < laser2d_config.size(); ++i)
                    {
                        const nlohmann::json &j_laser2d_instance = laser2d_config[i];
                        //std::cout << j_laser2d_instance["NAME"] << "\n";
                        mp_params->laser_2d_list.push_back(j_laser2d_instance["NAME"]);
                        mp_params->laser_2d_update_cycle_time_ms[j_laser2d_instance["NAME"]] =
                            j_laser2d_instance["UPDATE_CYCLE_TIME_MS"];
                    }
                }
                std::ifstream i_laser_3d_file(laser_3d_config_file);
                if(i_laser_3d_file)
                {
                    nlohmann::json j_laser_3d;
                    i_laser_3d_file >> j_laser_3d;
                    //std::cout << j_laser_3d["LASER_3D"] << "\n";
                    std::vector<nlohmann::json> laser3d_config = j_laser_3d["LASER_3D"];
                    for(int i = 0; i < laser3d_config.size(); ++i)
                    {
                        const nlohmann::json &j_laser3d_instance = laser3d_config[i];
                        //std::cout << j_laser3d_instance["NAME"] << "\n";
                        mp_params->laser_3d_list.push_back(j_laser3d_instance["NAME"]);
                        mp_params->laser_3d_update_cycle_time_ms[j_laser3d_instance["NAME"]] =
                            j_laser3d_instance["UPDATE_CYCLE_TIME_MS"];
                    }
                }
            }

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


bool TaskSensor::update_laser2d(const std::string& name)
{
    //PRINT_DEBUG("name: {}", name);
    bool r = mp_params->p_client_robot->connect();
    if (!r)
    {
        PRINT_ERROR("connect robot server failed");
        return false;
    }
    ReturnStatus ret_status = mp_params->p_client_robot->call<ReturnStatus>("update_laser2d", name);
    //PRINT_DEBUG("ret_status: {}", ret_status);
    if(ret_status == RET_SUCCESS)
    {
        RetMsg<LaserData> laser2d_data = mp_params->p_client_robot->call<RetMsg<LaserData>>("laser2d_data", name);
        mp_params->p_client_map->call("set_laser2d_data", laser2d_data.msg);
    }
    return true;
}

bool TaskSensor::update_laser3d(const std::string& name)
{
    //PRINT_DEBUG("update local map start");
    bool r = mp_params->p_client_robot->connect();
    if (!r)
    {
        PRINT_ERROR("connect robot server failed");
        return false;
    }
    ReturnStatus ret_status = mp_params->p_client_robot->call<ReturnStatus>("update_laser3d", name);
    if(ret_status == RET_SUCCESS)
    {
        RetMsg<LaserData> laser3d_data = mp_params->p_client_robot->call<RetMsg<LaserData>>("laser3d_data", name);
        mp_params->p_client_map->call("set_laser3d_data", laser3d_data.msg);
    }

    return true;
}

bool TaskSensor::register_background_tasks()
{
    //PRINT_DEBUG();
    timer::Timer& t = timer::GetTimer::timer();
    //PRINT_DEBUG("mp_params->laser_2d_list size = {}", mp_params->laser_2d_list.size());
    for(auto it = mp_params->laser_2d_list.begin(); it != mp_params->laser_2d_list.end(); ++it)
    {
        const uint32_t cycle_time_ms = mp_params->laser_2d_update_cycle_time_ms[*it];
        std::string *p_str = new std::string(*it);
        mp_params->p_task_params_laser_2d.emplace_back(p_str);
        t.add_task("task_update_laser2d", cycle_time_ms, &TaskSensor::task_update_laser2d, this, p_str);
    }

    for(auto it = mp_params->laser_3d_list.begin(); it != mp_params->laser_3d_list.end(); ++it)
    {
        const uint32_t cycle_time_ms = mp_params->laser_3d_update_cycle_time_ms[*it];
        std::string *p_str = new std::string(*it);
        mp_params->p_task_params_laser_3d.emplace_back(p_str);
        t.add_task("task_update_laser3d", cycle_time_ms, &TaskSensor::task_update_laser3d, this, p_str);
    }

    return true;

}

inline TaskSensorParams * const TaskSensor::params() const
{
    return mp_params.get();
}

Msg<Pose<FLOAT_T> > TaskSensor::location()
{

    bool r = mp_params->p_client_localization->connect();
    if(r)
    {
        return mp_params->p_client_localization->call<RetMsg<Pose<FLOAT_T>>>("location").msg;
    }
    else
    {
        PRINT_ERROR("connect map server failed");

    }
    return Msg<Pose<FLOAT_T>>();
}

void TaskSensor::task_update_laser2d(timer::TimerTaskItem *p_task, void *p_args)
{
    const std::string &name = *(std::string*)(p_args);
    const uint32_t update_cycle_time_ms = mp_params->laser_2d_update_cycle_time_ms[name];
    p_task->add_task(update_cycle_time_ms, p_args);
    update_laser2d(name);
}

void TaskSensor::task_update_laser3d(timer::TimerTaskItem *p_task, void *p_args)
{
    const std::string &name = *(std::string*)(p_args);
    const uint32_t update_cycle_time_ms = mp_params->laser_3d_update_cycle_time_ms[name];
    p_task->add_task(update_cycle_time_ms, p_args);
    update_laser3d(name);
}

}
