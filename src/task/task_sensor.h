#pragma once

#include <memory>
#include "task_base.h"
#include "common/data_types.h"

namespace bz_robot
{
namespace timer
{
    class TimerTaskItem;
}

class TaskSensorParams;
class TaskSensor: public TaskBase
{
public:
    TaskSensor();
    ~TaskSensor();
    bool import_config(const char* file_path);
    //bool init();
    //bool set_cmd(const char* cmd);
    //bool run();
    //bool stop();
    bool update_laser2d(const std::string &name);
    bool update_laser3d(const std::string &name);
    bool register_background_tasks();
    inline TaskSensorParams * const params() const;
private:
    inline Msg<Pose<FLOAT_T>> location();
    void task_update_laser2d(timer::TimerTaskItem *p_task, void* p_args);
    void task_update_laser3d(timer::TimerTaskItem *p_task, void* p_args);
private:
    std::shared_ptr<TaskSensorParams> mp_params;
};

}
