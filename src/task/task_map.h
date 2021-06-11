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

class TaskMapParams;
class TaskMap: public TaskBase
{
public:
    TaskMap();
    ~TaskMap();
    bool import_config(const char* file_path);
    //bool init();
    //bool set_cmd(const char* cmd);
    //bool run();
    //bool stop();
//    bool update_global_map();
//    bool update_local_map();
//    void thread_update_global_map();
//    void thread_update_local_map();
    bool register_background_tasks();
    inline const TaskMapParams* const params() const;
private:
    void update_global_map(timer::TimerTaskItem *p_task, void *p_params);
    void update_local_map(timer::TimerTaskItem *p_task, void *p_params);


    inline Msg<Pose<FLOAT_T>> location();
private:
    std::shared_ptr<TaskMapParams> mp_params;
};

}
