#pragma once

#include <memory>
#include "task_base.h"
#include "common/data_types.h"

namespace bz_robot
{

class MapBase;
class TaskNavigationParams;
class GridMapData;
class TaskNavigation: public TaskBase
{
public:
    TaskNavigation();
    ~TaskNavigation();
    bool import_config(const char* file);
    bool init();
    //std::shared_ptr<TaskNavigationParams> params();
    TaskNavigationParams* params();
    bool set_cmd(const std::string &cmd);
    RetMsg<std::string> run();
    bool stop();
    bool global_plan();
    bool local_plan();
    bool track_path();
private:
    inline GridMapData global_map();
    inline GridMapData local_map();
    inline Msg<Pose<FLOAT_T>> odom();
    void thread_global_plan();
    void thread_local_plan();
    void thread_tracker();

    const PathData& select_best_path(std::shared_ptr<bz_robot::MapBase> p_map, const PathData& path1,
                                     const PathData& path2, const Pose<float> &cur_pose);
    inline Msg<ControlData> control_data();
    bool is_arrive_goal(const Pose<FLOAT_T> &cur_pose, const Pose<FLOAT_T> &goal,
                        const FLOAT_T& distance_diff_tolerance, const FLOAT_T& angle_diff_tolerance);
    //std::shared_ptr<TaskNavigationParams> mp_params;
    TaskNavigationParams* mp_params;
};

}
