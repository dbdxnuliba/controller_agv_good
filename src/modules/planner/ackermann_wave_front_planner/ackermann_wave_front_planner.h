#pragma once

#include <stdint.h>
#include <iostream>
#include <ctime>
#include <thread>
#include <mutex>
#include <vector>
#include <map>
#include <unordered_map>
#include <memory>
#include "common/geometry.h"
#include "planner_base.h"
#include "map_base.h"
//#include "path_smoother.h"
#include "fmm_planner.h"
//#include "smoother/sample_optimizing_smoother/path_optimizer.hpp"

namespace bz_robot
{

namespace ackermann_wave_front_planner
{
    //cost, x, y, heading_index
    typedef float Node[4];
    typedef enum
    {
        I_COST = 0,
        I_X = 1,
        I_Y = 2,
        I_HEADING = 3,
        I_IS_CLOSED = 4,
        I_TOTAL
    }Index;



    class WaveFrontPlanner: public PlannerBase
    {
    public:
        WaveFrontPlanner();
        bool plan(std::shared_ptr<MapBase> p_map, Pose<float> pose_start,
                  Pose<float> pose_goal, std::vector<Pose<float>> *p_path);
        bool import_config(const char* config_file);
    private:
        void default_config();

        bool wave_front_explore(const Pose<float> &start_pose,
                             const Pose<float> &goal_pose,
                             std::vector<Pose<float> > *p_path);

        bool fmm_explore(const Pose<float> &start_pose,
                         const Pose<float> &goal_pose,
                         std::vector<Pose<float> > *p_path);
        bool sorted_wave_front(Pose<float> start_pose,
                        Pose<float> goal_pose,
                        std::vector<Pose<float> > *p_path);
        //
        void cache_look_up_table(const float resolution);
        inline uint64_t pose_to_index(const uint32_t x, const uint32_t y, const uint32_t heading_index);
        inline uint64_t pose_to_index(const Pose<int> pose);
        inline Pose<int> index_to_pose(uint64_t index);
        void generate_path(uint64_t index,
                        std::vector<Pose<float> > *p_path);

        bool wave_front(Pose<float> start_pose,
                               Pose<float> goal_pose,
                               std::vector<Pose<float> > *p_path);
        inline bool check_obstacles(const uint32_t x, const uint32_t y, const int dx, const int dy);

        bool sorted_wave_front2(Pose<float> start_pose,
                               Pose<float> goal_pose,
                               std::vector<Pose<float>> *p_path);
        //全方向运动的路径，不考虑转弯半径
        bool find_path(Pose<float> start_pose,
                       Pose<float> goal_pose,
                       std::vector<Pose<float> > *p_path);
        void flow_field(const std::vector<std::vector<int> > &costmap,
                        const VectorX2<int> &start_pose,
                        std::vector<Pose<float> > *p_path);
        void inflate_path(const std::vector<Pose<float>> &original_path);


        bool sorted_wave_front3(Pose<float> start_pose,
                               Pose<float> goal_pose,
                               std::vector<Pose<float> > *p_path);
        void stop();
    private:
        std::shared_ptr<MapBase> mp_map;

        std::vector<std::vector<int>> m_wave_front_look_up_table;
        std::vector<std::vector<float>> m_fmm_look_up_table;
//        std::unordered_map<uint64_t, float[5]> m_wave_map;
//        std::unordered_map<uint64_t, uint64_t> m_wave_front_prev_node;

        std::map<uint64_t, float[5]> m_wave_map;
        std::map<uint64_t, uint64_t> m_wave_front_prev_node;

        //std::map<Pose<int>, Pose<int>> m_path_list;
        Pose<float> m_start_pose;
        Pose<float> m_goal_pose;

        class MoveStep
        {
        public:
            MoveStep(const float &dx, const float &dy, const float &dh, const float &steer):
                delta_x(dx),
                delta_y(dy),
                length(hypot(dx, dy)),
                delta_heading(dh),
                steer_angle(steer)
            {

            }
            float delta_x;
            float delta_y;
            float length;
            float delta_heading;
            float steer_angle;
        };

        std::vector<std::vector<MoveStep>> m_look_up_table;//
        float m_resolution;//地图分辨率
        uint32_t m_map_w;//地图宽度
        uint32_t m_map_h;//地图高度

        uint32_t m_header_size;
        uint32_t m_steer_angle_size;
        float m_angle_step;//角度分辨率
        uint32_t m_distance_cost_ratio;
        uint32_t m_step_size;
        uint32_t m_step_cost_ratio;
        float m_delta_heading_cost_ratio;//
        float m_map_cost_ratio;//代价地图分辨率？？？？？？？
        float m_akermann_max_steer_angle;//允许的最大车头偏向角
        float m_akermann_wheel_base;//轮间距
        bool m_fast_mode;//快速模式，某些环节可以减小计算量？？？？？？？？？
        bool m_is_enable_smooth;//是否进行平滑？？？？？？？？？？？
        uint32_t m_smooth_times;//平滑的次数？？？？？？？？？

        std::vector<std::vector<int>> m_ref_path_costmap;
        std::vector<std::vector<std::vector<int>>> m_ref_path_3dcostmap;
        //PathSmoother m_path_smooth;
        fmm_planner::FMMPlanner m_fmm_planner;
    };
};
}

