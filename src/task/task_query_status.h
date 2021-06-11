#pragma once


#include <memory>
//#include "task_base.h"
#include "common/data_types.h"
#include "map/grid_map_data.h"
///#include "common/timer/timer.h"

namespace bz_robot
{

class TaskQueryStatusParams;
class TaskQueryStatus
{
public:
    TaskQueryStatus();
    ~TaskQueryStatus();

    bool safe_drive_status() const;
    std::string robot_name() const;
    std::string map_name() const;
    GridMapData global_map() const;
    PathData global_path() const;
    PathData local_path() const;
    int localization_status() const;
    bool motor_state() const;
    void cur_task() const;
    std::vector<uint8_t> input() const;
    std::vector<uint8_t> output() const;
    PathData path() const;
    RobotInfo robot_info() const;
    void robot_size() const;
    
private:
    std::shared_ptr<TaskQueryStatusParams> mp_params;
};

}
