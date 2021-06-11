#pragma once


#include <memory>
#include "task_base.h"
#include "common/data_types.h"
///#include "common/timer/timer.h"

namespace bz_robot
{
namespace timer
{
    class TimerTaskItem;
}

class TaskDriveParams;
class TaskDrive: public TaskBase
{
public:
    TaskDrive();
    ~TaskDrive();
    bool import_config(const char* file_path);
    bool set_cmd(const std::string &cmd);
    RetMsg<std::string> run();
    bool register_background_tasks();
private:
    void add_monitor();
    void timer_monitor_velocity(timer::TimerTaskItem *p_task, void *p_params);
private:
    std::shared_ptr<TaskDriveParams> mp_params;
};

}
