#pragma once


#include <iostream>
#include <vector>
#include "common/geometry.h"
#include "cubic_spline_2d.h"
#include "modules/map/map_base.h"
#include "modules/planner/planner_base.h"

namespace bz_robot
{

class FrenetOptimalPlanner : public PlannerBase
{
    class Path
    {
        public:
            Path()
            {
                t.clear();
                d.clear();
                d_d.clear();
                d_dd.clear();
                d_ddd.clear();
                s.clear();
                s_d.clear();
                s_dd.clear();
                s_ddd.clear();
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
            };
        public:
            std::vector<float> t;
            std::vector<float> d;
            std::vector<float> d_d;
            std::vector<float> d_dd;
            std::vector<float> d_ddd;
            std::vector<float> s;
            std::vector<float> s_d;
            std::vector<float> s_dd;
            std::vector<float> s_ddd;

            std::vector<float> x;
            std::vector<float> y;
            std::vector<float> yaw;
            std::vector<float> ds;
            std::vector<float> c;

            float c_lateral_deviation = 0.0;
            float c_lateral_velocity = 0.0;
            float c_lateral_acceleration = 0.0;
            float c_lateral_jerk = 0.0;
            float c_lateral = 0.0;

            // longitudinal costs
            float c_longitudinal_acceleration = 0.0;
            float c_longitudinal_jerk = 0.0;
            float c_time_taken = 0.0;
            float c_end_speed_deviation = 0.0;
            float c_longitudinal = 0.0;

            // obstacle costs
            float c_inv_dist_to_obstacles = 0.0;

            // final cost
            float cf = 0.0;
            float cd = 0.0;
            float cv = 0.0;

    };
public:
    FrenetOptimalPlanner();
    bool plan(std::shared_ptr<MapBase> p_map, Pose<float> start_pose,
              Pose<float> goal_pose, std::vector<Pose<float> > *p_path);
    bool import_config(const char* config_file);
private:
    void default_config();
    std::vector<Path> calc_frenet_paths(const float c_speed, const float c_d,
                                        const float c_d_d, const float c_d_dd, const float s0,
                                        CubicSpline2D *p_csp);
    bool calc_global_path(Path *p_path, CubicSpline2D *p_csp);
    bool select_best_path(std::vector<Path> *p_path_list, CubicSpline2D *p_csp, Path *p_best_path);
private:
    std::shared_ptr<MapBase> mp_map;
    float m_max_road_width;
    float m_road_width_step;
    float m_min_prediction_time;
    float m_max_prediction_time;
    float m_dt;
    float m_target_speed;
    float m_speed_step;
    float m_speed_step_number;
    float m_max_curvature;

    float m_kd;
//    float m_kv;
//    float m_ka;
    float m_kj;
    float m_kt;
//    float m_ko;
    float m_klat;
    float m_klon;
    std::vector<float> m_global_path_x;
    std::vector<float> m_global_path_y;
    std::vector<float> m_global_path_heading_angle;
    std::vector<float> m_global_path_frenet_s;
    std::vector<float> m_global_path_frenet_d;

    Pose<float> m_map_pose_start;
    bool m_is_use_csp;
};
}
