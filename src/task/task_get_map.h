#pragma once

#include "task_base.h"
#include "common/data_types.h"
#include "map/grid_map_interface.h"

namespace bz_robot
{
class MapBase;
class GridMapData;
class TaskGetMap: public TaskBase
{
public:
    TaskGetMap()
    {
    }
    ~TaskGetMap();
    // bool import_config(const char* file);
    bool init();
    // inline const TaskNavigationParams* const params() const;
    // bool set_cmd(const std::string &cmd);
    RetMsg<std::string> run()
    {
        RetMsg<std::string> ret_msg;
        ret_msg.return_status = RET_FAILED;
//	    bool r = mp_params->p_client_map->connect(ceil(mp_params->global_map_periodic_time_ms * 2 * 0.001));
//	    if (!r)
//	    {
//	        PRINT_ERROR("connect global planner server failed");
//	        return GridMapData();
//	    }
        const RetMsg<GridMapData> map_data = std::move(mp_params->p_client_map->call<RetMsg<GridMapData>>("global_map"));
        if(map_data.return_status == RET_SUCCESS)
        {
            GridMapInterface map(map_data.msg.data);
            ret_msg.msg.data = std::move(map.map_to_um_json());
            ret_msg.return_status = RET_SUCCESS;
        }
        return ret_msg;
    }

    bool stop();
    // bool global_plan();
    // bool local_plan();
    // bool track_path();
private:
    // inline GridMapData global_map();
    // inline GridMapData local_map();
    // inline Msg<Pose<FLOAT_T>> odom();
    // const PathData& select_best_path(std::shared_ptr<bz_robot::MapBase> p_map, const PathData& path1,
    //                                  const PathData& path2, const Pose<float> &cur_pose);
    // inline Msg<ControlData> control_data();

    // std::shared_ptr<TaskNavigationParams> mp_params;
};

}
