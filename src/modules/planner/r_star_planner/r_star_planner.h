#pragma once

#include <stdint.h>
#include <iostream>
#include <ctime>
#include <thread>
#include <mutex>
#include <vector>
#include <map>
#include <memory>
#include "common/geometry.h"
#include "map_base.h"
#include "planner_base.h"
//#include "modules/navigation/path_smoother/path_smoother.h"
#include "fmm_planner.h"

//class PathSmoother;
namespace bz_robot
{

namespace r_star_planner
{

    class RStarPlanner: public PlannerBase
    {
    public:
        RStarPlanner();
        bool plan(std::shared_ptr<MapBase> p_map, Pose<float> pose_start,
                  Pose<float> pose_goal, std::vector<Pose<float>> *p_path);
    private:

        void wave_front_explore(const Pose<float> &start_pose,
                                const Pose<float> &goal_pose);
        void wave_front_explore_3d(const Pose<float> &start_pose,
                                const Pose<float> &goal_pose);
        bool directed_wave_front(const Pose<float> &start_pose,
                                const Pose<float> &goal_pose,
                                std::vector<Pose<float> > *p_path);
        bool directed_astar(const Pose<float> &start_pose,
                                 const Pose<float> &goal_pose,
                                 std::vector<Pose<float> > *p_path);
        bool directed_astar2(const Pose<float> &start_pose,
                            const Pose<float> &goal_pose,
                            std::vector<Pose<float> > *p_path);
        std::vector<Pose<float>> smooth(std::vector<Pose<float>> *p_path,
                                         float weight_data = 0.5, float weight_smooth = 0.1, float tolerance = 0.000001);
    private:
        std::shared_ptr<MapBase> mp_map;
        std::vector<std::vector<int>> m_fast_wave_front_index;
        std::vector<std::vector<int>> m_wave_front_costmap;
        std::vector<std::vector<int>> m_wave_explore_costmap;
        std::vector<std::vector<float>> m_fmm_costmap;
        Pose<float> m_start_pose;
        Pose<float> m_goal_pose;
//        PathSmoother *mp_path_smooth;
        fmm_planner::FMMPlanner m_fmm_planner;
    };
};


}
