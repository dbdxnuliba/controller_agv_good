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

namespace bz_robot
{

namespace fmm_planner
{
    class FMMPlanner
    {
    public:
        FMMPlanner();
        bool plan(std::shared_ptr<MapBase> p_map, Pose<float> pose_start,
                  Pose<float> pose_goal, const uint32_t &over_time_ms, std::vector<Pose<float>> *p_path);
        bool fmm_explore(std::shared_ptr<MapBase> p_map, const Pose<float> &pose_start, const Pose<float> &pose_goal);
        std::vector<std::vector<float>> costmap();
    private:
        //求解二次方程
        inline float solve_quadratic(const uint32_t &x, std::vector<float> *tau);
        void flow_field(const std::vector<std::vector<float> > &costmap,
                        const VectorX2<int> &start_pose,
                        std::vector<Pose<float> > *p_path);
    private:
        std::shared_ptr<MapBase> mp_map;//地图类的指针
        Pose<float> m_start_pose;//起点坐标
        Pose<float> m_goal_pose;//目标点坐标
        std::vector<float> m_fmm_costmap;//地图栅格的代价值
        std::vector<std::vector<float>> m_costmap;//地图栅格的代价值
        std::vector<uint8_t> m_flags;//地图栅格状态量，包括unknown，know,front
        std::vector<float> m_speed_list;//速度图
        uint32_t m_map_w;//地图宽度，对应x轴
        uint32_t m_map_h;//地图高度，对应y轴
        float m_beta[2];
        float alpha_sq[2];
        bool m_skipped[2];
        uint32_t m_shift[2];//轮换过的网格数量， m_shift[0]存放一行的网格数，m_shift[1]为1
        uint32_t m_shape[2];//储存了地图的长度和宽度，整型
    };
};

}
