#include <map>
#include <list>
#include <mutex>
#include <fstream>
//#include <event2/event.h>
#include "common/common.h"
#include "common/json.hpp"
#include "common/print.h"
#include "common/periodic_task.h"
#include "common/thread_rpc.h"
#include "common/msg_id.h"
#include "common/time_keeper.h"
#include "task/task_manager.h"
#include "task/task_base.h"
#include "task/task_drive.h"
#include "task/task_map.h"
#include "task/task_navigation.h"
#include "task/task_sensor.h"
#include "task/task_stop.h"


namespace bz_robot
{

class TaskItem
{
public:
    TaskItem():p_task(nullptr) {}
    TaskBase* p_task = nullptr;
    std::string task_cmd;
};


class TaskManagerParams
{
public:
    TaskItem cur_task;
    std::list<TaskItem> p_task_list;
    std::map<TaskId, TaskBase*> p_tasks_map;
    std::recursive_mutex mtx;
    std::vector<std::thread*> thread_list;
    std::shared_ptr<thread_rpc::Client> p_client_robot;
    std::shared_ptr<thread_rpc::Client> p_client_map;
    std::shared_ptr<thread_rpc::Client> p_client_global_planner;
    std::shared_ptr<thread_rpc::Client> p_client_local_planner;
    std::shared_ptr<thread_rpc::Client> p_client_tracker;
    std::shared_ptr<thread_rpc::Client> p_client_localization;
    std::shared_ptr<thread_rpc::Client> p_client_global_smoother;
    std::shared_ptr<thread_rpc::Client> p_client_local_smoother;
    std::atomic<bool> flag_is_new_task;
};

inline static bool import_config(std::shared_ptr<thread_rpc::Client> p_client, const std::string& config_file)
{
    RECORD_TIME();
    bool r;
    //r = p_client->connect();
    //std::cout<<"读取参数文件："<<config_file<<std::endl;
    PRINT_INFO("读取参数文件： {}", config_file);
    p_client->call("import_config", config_file);
    return true;
}

TaskManager::TaskManager()
{
    mp_params = std::make_shared<TaskManagerParams>();
    init();
}

TaskManager::~TaskManager()
{

}

bool TaskManager::init()
{
    mp_params->p_client_robot = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_ROBOT));
    mp_params->p_client_map = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_MAP));
    mp_params->p_client_global_planner = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_GLOBAL_PLAN));
    mp_params->p_client_local_planner = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_LOCAL_PLAN));
    mp_params->p_client_tracker = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_TRACKER));
    mp_params->p_client_localization = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_LOCALIZATION));
    mp_params->p_client_global_smoother = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_GLOBAL_PATH_SMOOTH));
    mp_params->p_client_local_smoother = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_LOCAL_PATH_SMOOTH));

    mp_params->p_tasks_map[TaskId::MAP] = new TaskMap();
    mp_params->p_tasks_map[TaskId::NAV] = new TaskNavigation();
    mp_params->p_tasks_map[TaskId::SENSOR] = new TaskSensor();
    mp_params->p_tasks_map[TaskId::DRIVE] = new TaskDrive();
    mp_params->p_tasks_map[TaskId::STOP] = new TaskStop();
}

bool TaskManager::import_configs(const char *file_path)
{
    RECORD_TIME();
    PRINT_DEBUG("\n\n\n");
    bool result = false;
    try
    {
        std::ifstream i(file_path);
        if(i)
        {
            nlohmann::json j;
            i >> j;

            //std::cout << j << std::endl;
            PRINT_INFO("{}", j.dump(4));
            const std::string robot_config_file = j["ROBOT_CONFIG"];
            const std::string map_config_file = j["MAP_CONFIG"];
            const std::string global_planner_config_file = j["GLOBAL_PLANNER_CONFIG"];
            const std::string local_planner_config_file = j["LOCAL_PLANNER_CONFIG"];
            const std::string tracker_config_file = j["TRACKER_CONFIG"];
            const std::string global_smoother_config_file = j["GLOBAL_PATH_SMOOTHER_CONFIG"];
            const std::string local_smoother_config_file = j["LOCAL_PATH_SMOOTHER_CONFIG"];
            const std::string nav_config_file = j["NAVIGATION_CONFIG"];

            import_config(mp_params->p_client_robot, robot_config_file);
            import_config(mp_params->p_client_map, map_config_file);
            import_config(mp_params->p_client_global_planner, global_planner_config_file);
            import_config(mp_params->p_client_local_planner, local_planner_config_file);
            import_config(mp_params->p_client_tracker, tracker_config_file);
            import_config(mp_params->p_client_global_smoother, global_smoother_config_file);
            import_config(mp_params->p_client_local_smoother, local_smoother_config_file);
            PRINT_DEBUG("task manager finished import server config");
            //import tasks config
            mp_params->p_tasks_map[TaskId::MAP]->import_config(map_config_file);
            mp_params->p_tasks_map[TaskId::NAV]->import_config(nav_config_file);
            mp_params->p_tasks_map[TaskId::SENSOR]->import_config(robot_config_file);
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
    PRINT_DEBUG("finish import config\n");
    return result;
}

bool TaskManager::start()
{
    mp_params-> thread_list.push_back(new std::thread(&TaskManager::task_list_monitor, this));
    mp_params->thread_list.push_back(new std::thread(&TaskManager::running_background_tasks, this));
    mp_params->thread_list.push_back(new std::thread(&TaskManager::running_tasks, this));
    for(auto it = mp_params->thread_list.begin(); it != mp_params->thread_list.end(); ++it)
    {
        (*it)->detach();
    }
}

bool TaskManager::set_next_task(const TaskId& task_id, const std::string &argc)
{
    auto task_map_it = mp_params->p_tasks_map.find(task_id);
    if(mp_params->p_tasks_map.find(task_id) == mp_params->p_tasks_map.end())
    {
        PRINT_ERROR("unsupported task: {}", task_id);
        return false;
    }
    TaskItem task_item;
    task_item.p_task = mp_params->p_tasks_map[task_id];
    task_item.task_cmd = argc;
    PRINT_DEBUG("next task {}\n\n\n", task_item.task_cmd);
    std::lock_guard<std::recursive_mutex> temp_lock(mp_params->mtx);
    mp_params->p_task_list.push_back(task_item);
    mp_params->flag_is_new_task = true;
    return true;
}

bool TaskManager::stop_task(const TaskId &task_id)
{
    auto task_map_it = mp_params->p_tasks_map.find(task_id);
    if(mp_params->p_tasks_map.find(task_id) == mp_params->p_tasks_map.end())
    {
        PRINT_ERROR("unsupported task: {}", task_id);
        return false;
    }
    return task_map_it->second->stop();
}

void TaskManager::task_list_monitor()
{
    while (true)
    {
        PERIODIC_MS_TASK(100);
        {
            std::lock_guard<std::recursive_mutex> temp_lock(mp_params->mtx);
            //PRINT_DEBUG("mp_params->p_task_list.dize = {}", mp_params->p_task_list.size());
            if(!mp_params->p_task_list.empty())
            {
                printf("mp_params->cur_task.p_task = %p\n\n", mp_params->cur_task.p_task);
                if(mp_params->cur_task.p_task)
                {
                    PRINT_DEBUG("call task stop");
                    if(mp_params->cur_task.p_task->stop())
                    {
                        mp_params->cur_task = mp_params->p_task_list.back();
                        mp_params->p_task_list.clear();
                    }
                    else
                    {
                        PRINT_ERROR("cur task can't stop");
                    }
                }
                else
                {
                    PRINT_DEBUG("cur task == nullptr");
                    mp_params->cur_task = mp_params->p_task_list.back();
                    mp_params->p_task_list.clear();
                    //mp_params->p_task_list.erase(mp_params->p_task_list.begin(), mp_params->p_task_list.end()-1);
                }
                PRINT_DEBUG("set task {}\n\n\n", mp_params->cur_task.task_cmd);
            }
            else
            {
                //PRINT_DEBUG("no task, switch to idle task");
                //mp_params->cur_task.p_task = nullptr;
                //set next task to idle task;
                //mp_params->p_cur_task = nullptr;
            }
        }
    }
}

void TaskManager::running_background_tasks()
{
    //struct event_base* p_event_base = event_base_new();
    bool status = false;
    for(auto it = mp_params->p_tasks_map.begin(); it != mp_params->p_tasks_map.end(); ++it)
    {
        bool task_state = (it->second)->register_background_tasks();
        if(task_state)
        {
            status = true;
        }
    }
//    if(status)
//    {
//        int err = event_base_dispatch(p_event_base);
//        PRINT_ERROR("error = {}", err);
//    }

//    event_base_free(p_event_base);
}

bool TaskManager::running_tasks()
{
    const TaskItem empty_task;
    while (true)
    {
        //PERIODIC_MS_TASK(100);
        //TaskBase *p_task = nullptr;
        //PRINT_DEBUG("running tasks\n\n");
        TaskItem cur_task_item;
        {
            std::lock_guard<std::recursive_mutex> temp_lock(mp_params->mtx);
            cur_task_item = mp_params->cur_task;
            //mp_params->cur_task = empty_task;
        }
        if(cur_task_item.p_task != nullptr)
        {
            mp_params->flag_is_new_task = false;
            PRINT_DEBUG("run task {}\n\n\n", cur_task_item.task_cmd);
            cur_task_item.p_task->set_cmd(cur_task_item.task_cmd);
            cur_task_item.p_task->run();
            cur_task_item.p_task->stop();

            //std::lock_guard<std::recursive_mutex> temp_lock(mp_params->mtx);
            if(!mp_params->flag_is_new_task)
            {
                PRINT_DEBUG("switch into empty task");
                mp_params->cur_task = empty_task;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            //mp_params->cur_task.p_task = nullptr;
        }
        else
        {
            //PRINT_DEBUG("no task running");
            // switch to idel task
        }
        //set task to idle
        std::lock_guard<std::recursive_mutex> temp_lock(mp_params->mtx);
        if(mp_params->cur_task.p_task != nullptr)
        {
            //add idle task
        }

    }

}

RetMsg<std::string> TaskManager::run_task(const TaskId &task_id, const std::string &argc)
{
    RetMsg<std::string> ret_msg;
    auto task_map_it = mp_params->p_tasks_map.find(task_id);
    if(task_map_it == mp_params->p_tasks_map.end())
    {
        PRINT_ERROR("unsupported task: {}", task_id);
        ret_msg.return_status = RET_ERROR;
        return ret_msg;
    }
    task_map_it->second->set_cmd(argc);
    ret_msg = task_map_it->second->run();
    return ret_msg;
}

}
