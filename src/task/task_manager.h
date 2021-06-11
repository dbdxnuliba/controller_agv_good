#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <atomic>
#include <thread>
#include "common/data_types.h"
#include "task/task_id.h"

namespace bz_robot
{


class TaskBase;
class TaskManagerParams;
class TaskManager
{
public:
    TaskManager();
    ~TaskManager();

    bool import_configs(const char* file_path);
    bool import_configs(const std::string &file_path) {return import_configs(file_path.c_str());}
    bool start();
    bool set_next_task(const TaskId &task_id, const std::string& argc);
    bool stop_task(const TaskId &task_id);
    RetMsg<std::string> run_task(const TaskId &task_id, const std::string& argc);
private:
    std::shared_ptr<TaskManagerParams> mp_params;
private:
    bool init();
    void running_background_tasks();
    bool running_tasks();
    void task_list_monitor();
};
}
//#endif //__SERVICE_MANAGER__
