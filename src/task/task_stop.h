#pragma once

#include "task_base.h"
#include "common/data_types.h"
#include "map/grid_map_interface.h"

namespace bz_robot
{
class TaskStopParams;
class TaskStop: public TaskBase
{
public:
    TaskStop();
    ~TaskStop();

    RetMsg<std::string> run();
private:
	std::shared_ptr<TaskStopParams> mp_params;
};

}
