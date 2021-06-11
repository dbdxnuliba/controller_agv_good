#pragma once


#include <iostream>
#include <vector>
#include <limits>
#include "common/geometry.h"
#include "modules/planner/planner_base.h"

namespace bz_robot
{
namespace frenet_polynomial_planner
{
class CubicSpline2D;

class FrenetPolynomialPlanner : public PlannerBase
{
    class Path
    {
        public:
            Path()
            {
                t.clear();
                d.clear();
                d_position.clear();
                d_v.clear();
                d_acc.clear();
                s.clear();
                s_position.clear();
                s_v.clear();
                s_acc.clear();
                x.clear();
                y.clear();
                yaw.clear();
                ds.clear();
                c.clear();
                c_lateral_deviation = 0.0;
                c_lateral_velocity = 0.0;
                c_lateral_acceleration = 0.0;
                c_lateral_jerk = 0.0;
                c_lateral = 0.0;
                // longitudinal costs
                c_longitudinal_acceleration = 0.0;
                c_longitudinal_jerk = 0.0;
                c_time_taken = 0.0;
                c_end_speed_deviation = 0.0;
                c_longitudinal = 0.0;
                // obstacle costs
                c_inv_dist_to_obstacles = 0.0;
                // final cost
                cf = 0.0;
                cd = 0.0;
                cv = 0.0;
                score = std::numeric_limits<float>::max();
            }
            void resize(const uint32_t& size)
            {
                t.resize(size);
                d.resize(size);
                d_position.resize(size);
                d_v.resize(size);
                d_acc.resize(size);
                s.resize(size);
                s_position.resize(size);
                s_v.resize(size);
                s_acc.resize(size);

                x.resize(size);
                y.resize(size);
                yaw.resize(size);
                ds.resize(size);
                c.resize(size);
            }
        public:
            std::vector<FLOAT_T> t;
            std::vector<FLOAT_T> d;
            std::vector<FLOAT_T> d_position;
            std::vector<FLOAT_T> d_v;
            std::vector<FLOAT_T> d_acc;
            std::vector<FLOAT_T> s;
            std::vector<FLOAT_T> s_position;
            std::vector<FLOAT_T> s_v;
            std::vector<FLOAT_T> s_acc;

            std::vector<FLOAT_T> x;
            std::vector<FLOAT_T> y;
            std::vector<FLOAT_T> yaw;
            std::vector<FLOAT_T> ds;
            std::vector<FLOAT_T> c;

            FLOAT_T c_lateral_deviation = 0.0;
            FLOAT_T c_lateral_velocity = 0.0;
            FLOAT_T c_lateral_acceleration = 0.0;
            FLOAT_T c_lateral_jerk = 0.0;
            FLOAT_T c_lateral = 0.0;

            // longitudinal costs
            FLOAT_T c_longitudinal_acceleration = 0.0;
            FLOAT_T c_longitudinal_jerk = 0.0;
            FLOAT_T c_time_taken = 0.0;
            FLOAT_T c_end_speed_deviation = 0.0;
            FLOAT_T c_longitudinal = 0.0;

            // obstacle costs
            FLOAT_T c_inv_dist_to_obstacles = 0.0;

            // final cost
            FLOAT_T cf = 0.0;
            FLOAT_T cd = 0.0;
            FLOAT_T cv = 0.0;

            FLOAT_T score;

    };
public:
    FrenetPolynomialPlanner();
    bool plan(std::shared_ptr<MapBase> p_map, Pose<FLOAT_T> start_pose,
              Pose<FLOAT_T> goal_pose, std::vector<Pose<FLOAT_T> > *p_path);
    bool import_config(const char* config_file);
private:
    void default_config();
    bool calc_frenet_path(const FLOAT_T &s_start_position,
                                        const FLOAT_T &d_start_position,
                                        CubicSpline2D *p_csp, Path *p_path);
    bool calc_frenet_path2(const FLOAT_T &s_start_position,
                                        const FLOAT_T &d_start_position,
                                        CubicSpline2D *p_csp, Path *p_path);
    bool calc_path_score(Path *p_path);
    FLOAT_T calc_curvature(const Pose<FLOAT_T>& p1, const Pose<FLOAT_T>& p2);
//    bool calc_global_path(Path *p_path, CubicSpline2D *p_csp);
//    bool select_best_path(std::vector<Path> *p_path_list, CubicSpline2D *p_csp, Path *p_best_path);
    void send_path_list_to_ros(const std::vector<Path>& path_list);
private:
    std::shared_ptr<MapBase> mp_map;
    FLOAT_T m_max_road_width;
    FLOAT_T m_road_width_step;
    FLOAT_T m_min_prediction_time;
    FLOAT_T m_max_prediction_time;
    FLOAT_T m_dt;
    FLOAT_T m_target_speed;
    FLOAT_T m_speed_step;
    FLOAT_T m_speed_step_number;
    FLOAT_T m_max_curvature;
    FLOAT_T m_goal_explore_range;
    FLOAT_T m_longitude_step;
    FLOAT_T m_kd;
//    FLOAT_T m_kv;
//    FLOAT_T m_ka;
    FLOAT_T m_kj;
    FLOAT_T m_kt;
//    FLOAT_T m_ko;
    FLOAT_T m_klat;
    FLOAT_T m_klon;
    std::vector<FLOAT_T> m_global_path_x;
    std::vector<FLOAT_T> m_global_path_y;
    std::vector<FLOAT_T> m_global_path_heading_angle;
    std::vector<FLOAT_T> m_global_path_frenet_s;
    std::vector<FLOAT_T> m_global_path_frenet_d;

    Pose<FLOAT_T> m_map_pose_start;
    bool m_is_use_csp;
};
}
}
