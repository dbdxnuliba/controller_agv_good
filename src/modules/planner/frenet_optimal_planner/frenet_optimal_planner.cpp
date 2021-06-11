#include "frenet_optimal_planner.h"
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <cmath>
#include <fstream>
#include <queue>
#include "quartic_polynomial.h"
#include "common/json.hpp"
#include "common/common.h"
#include "common/print.h"
#include "common/time_keeper.h"
#include "frenet_cartesian.h"


namespace bz_robot
{


FrenetOptimalPlanner::FrenetOptimalPlanner()
{
    default_config();
}

bool FrenetOptimalPlanner::plan(std::shared_ptr<MapBase> p_map, Pose<float> start_pose,
                                Pose<float> goal_pose, std::vector<Pose<float> > *p_path)
{
    //RECORD_TIME();
    const std::vector<Pose<float> > &global_path = m_ref_path;
    mp_map = p_map;

    int global_path_size = global_path.size();
    if(global_path_size < 2)
    {
        PRINT_ERROR("global_path_size < 2");
        return false;
    }

    m_global_path_x.clear();
    m_global_path_y.clear();
    m_global_path_heading_angle.clear();
    m_global_path_frenet_s.clear();
    m_global_path_frenet_d.clear();
    float mx = 0;
    float my = 0;
//    uint32_t mx = 0;
//    uint32_t my = 0;


    for(int i = 0; i < global_path_size; ++i)
    {
        if(mp_map->world_to_map(global_path[i].position.x, global_path[i].position.y, &mx, &my))
        {
            if(i == 0)
            {
                m_global_path_x.emplace_back((float)mx);
                m_global_path_y.emplace_back((float)my);
                m_global_path_heading_angle.emplace_back(global_path[i].heading_angle);
            }
            else
            {
                if(round(mx * 100) != round(m_global_path_x.back() * 100) ||
                   round(my * 100) != round(m_global_path_y.back() * 100))
                {
                    m_global_path_x.emplace_back((float)mx);
                    m_global_path_y.emplace_back((float)my);
                    m_global_path_heading_angle.emplace_back(global_path[i].heading_angle);
                }
            }
        }
        else
        {
            PRINT_ERROR("world pose({:.3f}, {:.3f}) to map error!", global_path[i].position.x, global_path[i].position.y);
        }
    }
    if(mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &mx, &my))
    {
        m_map_pose_start = Pose<float>(mx, my, start_pose.heading_angle);
    }
    else
    {
        return false;
    }

    CubicSpline2D csp = CubicSpline2D(m_global_path_x, m_global_path_y);
    for(int i = 0; i < m_global_path_x.size(); ++i)
    {
        std::vector<float> frenet_s_d = get_frenet(m_global_path_x[i], m_global_path_y[i], m_global_path_heading_angle[i],
                                                    m_global_path_x, m_global_path_y);
        m_global_path_frenet_s.emplace_back(frenet_s_d[0]);
        m_global_path_frenet_d.emplace_back(frenet_s_d[1]);
        //PRINT_DEBUG("global path({:.3f}, {:.3f}) => sd({:.3f}, {:.3f})", m_global_path_x[i], m_global_path_y[i], frenet_s_d[0], frenet_s_d[1]);
    }

    std::vector<float> frenet_start_pose = get_frenet(m_map_pose_start.position.x, m_map_pose_start.position.y,
                                                       m_map_pose_start.heading_angle,
                                                       m_global_path_x, m_global_path_y);

//    PRINT_DEBUG("cur pose({:.3f}, {:.3f}, {:.3f}) => sd({:.3f}, {:.3f})",
//                m_map_pose_start.position.x, m_map_pose_start.position.y,
//                radian_to_degree(m_map_pose_start.heading_angle),
//                frenet_start_pose[0], frenet_start_pose[1]);

    //float s0 = 0.0;  //current course position
    float s0 = frenet_start_pose[0];
    //float c_d = 0.0;  //current lateral position [m]
    float c_d = frenet_start_pose[1];  //current lateral positio
    float c_speed = m_target_speed; //current speed [m/s]
    float c_d_d = 0.0;  //current lateral speed [m/s]
    float c_d_dd = 0.0;  //current latral acceleration [m/s]

    std::vector<FrenetOptimalPlanner::Path> path_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, &csp);
    //PRINT_INFO("total path {:d}", path_list.size());
    Path best_path;
    if(select_best_path(&path_list, &csp, &best_path))
    {
        p_path->clear();
        p_path->resize(best_path.x.size());
        float wx = 0;
        float wy = 0;
        for(int i = 0; i < best_path.x.size(); ++i)
        {
//            PRINT_INFO("best path [{}]  sd:{:.3f}, {:.3f} => {:.3f}, {:.3f}, {:.1f}",
//                       i, best_path.s[i], best_path.d[i], best_path.x[i], best_path.y[i], radian_to_degree(best_path.yaw[i]));
            if(mp_map->map_to_world(best_path.x[i], best_path.y[i], &wx, &wy))
            {
                (*p_path)[i].position.x = wx;
                (*p_path)[i].position.y = wy;
                (*p_path)[i].heading_angle = best_path.yaw[i];
            }
            else
            {
                PRINT_ERROR("map pose({:.3f}, {:.3f}) to world error!", best_path.x[i], best_path.y[i]);
            }
        }
        (*p_path)[0] = global_path[0];
    }
    else
    {
        PRINT_WARN("frenet plan failed!!");
    }

    return true;
}

bool FrenetOptimalPlanner::import_config(const char *config_file)
{
    try
    {
        PRINT_INFO("read config: {}", config_file);
        std::ifstream i(config_file);
        if(i)
        {
            nlohmann::json j;
            i >> j;
            float max_road_width = j["max_road_width"];
            float road_width_step = j["road_width_step"];
            float min_prediction_time = j["min_prediction_time"];
            float max_prediction_time = j["max_prediction_time"];
            float time_step = j["time_step"];
            float target_speed = j["target_speed"];
            float speed_step = j["speed_step"];
            float speed_step_number = j["speed_step_number"];
            float max_curvature = j["max_curvature"];
            float kd = j["kd"];
            float kj = j["kj"];
            float kt = j["kt"];
            float klat = j["klat"];
            float klon = j["klon"];

            m_max_road_width = max_road_width;
            m_road_width_step = road_width_step;
            m_min_prediction_time = min_prediction_time;
            m_max_prediction_time = max_prediction_time;
            m_dt = time_step;
            m_target_speed = target_speed;
            m_speed_step = speed_step;
            m_speed_step_number = speed_step_number;
            m_max_curvature = max_curvature;
            m_kd = kd;
            m_kj = kj;
            m_kt = kt;
            m_klat = klat;
            m_klon = klon;
            return true;
        }
        else
        {
            PRINT_ERROR("can't read config files from: {}", config_file);
        }
    }
    catch(std::exception& e )
    {
        PRINT_ERROR("%s", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("un expexted occured");
    }

    return false;
}

void FrenetOptimalPlanner::default_config()
{
    m_max_road_width = 3.0;
    m_road_width_step = 1.0;
    m_min_prediction_time = 8.0;
    m_max_prediction_time = 10.0;
    m_dt = 0.5;
    m_target_speed = 2.0;
    m_speed_step = 0.5;
    m_speed_step_number = 3;
    m_max_curvature = 0.5;
    m_kd = 1.0;
//    m_kv = 0.1;
//    m_ka = 0.1;
    m_kj = 0.1;
    m_kt = 0.1;
//    m_ko = 0.1;
    m_klat = 1.0;
    m_klon = 1.0;
    m_is_use_csp = false;
}

std::vector<FrenetOptimalPlanner::Path> FrenetOptimalPlanner::calc_frenet_paths(const float c_speed, const float c_d,
                                                                                const float c_d_d, const float c_d_dd,
                                                                                const float s0, CubicSpline2D *p_csp)
{
    std::vector<FrenetOptimalPlanner::Path> path_list;
    path_list.clear();
    float ti, tv;
//    float lateral_deviation, lateral_velocity, lateral_acceleration, lateral_jerk;
//    float longitudinal_acceleration, longitudinal_jerk;
    //横向动作规划
    const float max_road_width = m_max_road_width / mp_map->resolution();
    const float road_width_step = m_road_width_step / mp_map->resolution();
    for(float di  = -max_road_width; di <= max_road_width; di += road_width_step)
    {
        for(ti = m_min_prediction_time; ti < m_max_prediction_time; ti += m_dt)
        {
//            lateral_deviation = 0;
//            lateral_velocity = 0;
//            lateral_acceleration = 0;
//            lateral_jerk = 0;

            Path path = Path();
            QuarticPolynomial6 lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, ti);

            const uint32_t size = floor(ti / m_dt);
            path.t.assign(size, 0);
            path.d.assign(size,0);
            path.d_d.assign(size,0);
            path.d_dd.assign(size,0);
            path.d_ddd.assign(size,0);
            path.s.assign(size,0);
            path.s_d.assign(size,0);
            path.s_dd.assign(size,0);
            path.s_ddd.assign(size,0);
            for(uint32_t i = 0; i < size; ++i)
            //for(float t = 0; t <=ti; t += m_dt)
            {
                float t = m_dt * i;
                float d = lat_qp.calc_point(t);
                float d_d = lat_qp.calc_first_derivative(t);
                float d_dd = lat_qp.calc_second_derivative(t);
                float d_ddd = lat_qp.calc_third_derivative(t);
                path.t[i] = t;
                path.d[i] = d;
                path.d_d[i] = d_d;
                path.d_dd[i] = d_dd;
                path.d_ddd[i] = d_ddd;
//                lateral_deviation += fabs(d);
//                lateral_velocity += fabs(d_d);
//                lateral_acceleration += fabs(d_dd);
//                lateral_jerk += fabs(d_ddd);
            }

            // Longitudinal motion planning (Velocity keeping)
            // 纵向速度规划 (速度保持)
            for(tv = m_target_speed - m_speed_step * m_speed_step_number;
                 tv < m_target_speed + m_speed_step * m_speed_step_number;
                 tv += m_speed_step)
            {
//                longitudinal_acceleration = 0;
//                longitudinal_jerk = 0;

                Path t_path = path;
                QuarticPolynomial5 lon_qp(s0, c_speed, 0, tv, 0.0, ti);
                float jp = 0;
                float js = 0;
                uint32_t i = 0;
                for(i = 0; i < size; ++i)
                {
                    float t = t_path.t[i];
                    float s = lon_qp.calc_point(t);
                    if(s > m_global_path_frenet_s.back())
                    {
                        break;
                    }
                    float s_d = lon_qp.calc_first_derivative(t);
                    float s_dd = lon_qp.calc_second_derivative(t);
                    float s_ddd = lon_qp.calc_third_derivative(t);
                    t_path.s[i] = s;
                    t_path.s_d[i] = s_d;
                    t_path.s_dd[i] = s_dd;
                    t_path.s_ddd[i] = s_ddd;
                    jp += t_path.d_ddd[i] * t_path.d_ddd[i];
                    js += t_path.s_ddd[i] * t_path.s_ddd[i];
//                    longitudinal_acceleration += fabs(s_dd);
//                    longitudinal_jerk += fabs(s_ddd);

                }
                t_path.s.erase(t_path.s.begin()+i, t_path.s.end());
                t_path.s_d.erase(t_path.s_d.begin()+i, t_path.s_d.end());
                t_path.s_dd.erase(t_path.s_dd.begin()+i, t_path.s_dd.end());
                t_path.s_ddd.erase(t_path.s_ddd.begin()+i, t_path.s_ddd.end());


                const float ds = pow((m_target_speed - t_path.s_d.back()), 2);
                t_path.cd = m_kj * jp + m_kt * ti + m_kd * pow(t_path.d.back(), 2);
                t_path.cv = m_kj * js + m_kt * ti + m_kd * ds;
                t_path.cf = m_klat * t_path.cd + m_klon * t_path.cv;
//                PRINT_DEBUG("ds = {:.3f}, Jp = {:.3f}, Js = {:.3f}, cd = {:.3f}, cv = {:.3f}, cf = {:.3f}",
//                            ds, jp, js, t_path.cd, t_path.cv, t_path.cf);
                path_list.push_back(t_path);
            }
        }
    }
    return path_list;
}

bool FrenetOptimalPlanner::calc_global_path(Path *p_path, CubicSpline2D *p_csp)
{
    float x = 0;
    float y = 0;
    float s = 0;
    float yaw = 0;
    float di = 0;
    float fx = 0;
    float fy = 0;
    float dx = 0;
    float dy = 0;
    float dyaw = 0;
    float c = 0;
    int j = 0;
    const float max_curvature = m_max_curvature / mp_map->resolution();
    int path_size = p_path->s.size();
    p_path->x.resize(path_size);
    p_path->y.resize(path_size);
    p_path->yaw.resize(path_size);
    p_path->ds.resize(path_size);
    p_path->c.resize(path_size);
    //printf("\n");
    for(j = 0; j < path_size; ++j)
    {
        s = p_path->s[j];
        di = p_path->d[j];

        if(m_is_use_csp)
        {
            x = p_csp->calc_x(s);
            y = p_csp->calc_y(s);

            if(std::isnan(x) || std::isnan(y))
            //if(std::isnan(x))
            {
                //PRINT_DEBUG("x {:.3f}, y: {:.3f}", x, y);
                break;
            }

            yaw = p_csp->calc_yaw(s);
            fx = x + di * cos(yaw + M_PI_2);
            fy = y + di * sin(yaw + M_PI_2);


            p_path->x[j] = fx;
            p_path->y[j] = fy;
        }
        else
        {

            //PRINT_DEBUG("s[{:d}] = %0.3f d[{:d}] = {:.3f}", j, s, j, di);
            std::vector<float> pos = get_xy(s, di, m_global_path_frenet_s, m_global_path_x, m_global_path_y);
//            if(s == 0.0)
//            {
//                pos[0] = m_map_pose_start.position.x;
//                pos[1] = m_map_pose_start.position.y;
//            }
            p_path->x[j] = pos[0];
            p_path->y[j] = pos[1];
            //p_path->yaw[j] = pos[3];
            if(s > m_global_path_frenet_s.back())
            {
                ++j;
                break;
            }
        }

        //PRINT_DEBUG("s = {:.3f}, d = {:.3f}, old: {:.3f}, {:.3f}, new {:.3f}, {:.3f}", s, p_path->d[j], fx, fy, pos[0], pos[1]);
        if(j > 0)
        {
            int i = j-1;
            dx = p_path->x[j] - p_path->x[i];
            dy = p_path->y[j] - p_path->y[i];
            p_path->yaw[i] = atan2(dy, dx);
            p_path->ds[i] = hypot(dy, dx);
            if(j < path_size - 1)
            {
                const FLOAT_T temp_dt0 = p_path->s[j] - p_path->s[j-1];
                const FLOAT_T temp_dt1 = p_path->s[j+1] - p_path->s[j];
                const FLOAT_T temp_dx0 = p_path->x[j] - p_path->x[j-1];
                const FLOAT_T temp_dy0 = p_path->y[j] - p_path->y[j-1];
                const FLOAT_T temp_dx1 = p_path->x[j+1] - p_path->x[j];
                const FLOAT_T temp_dy1 = p_path->y[j+1] - p_path->y[j];
                const FLOAT_T temp_sum_1 = 1.0f / temp_dt0 + temp_dt1;
                const FLOAT_T temp_dx = temp_dt0 * temp_sum_1 + temp_dx0 + temp_dt1 * temp_sum_1 * temp_dx1;
                const FLOAT_T temp_dy = temp_dt0 * temp_sum_1 + temp_dy0 + temp_dt1 * temp_sum_1 * temp_dy1;
                p_path->yaw[j] = atan2(temp_dy, temp_dx);
                //p_path->ds[j] = temp_dt0;
            }
            else
            {
//                int i = j-1;
//                dx = p_path->x[j] - p_path->x[i];
//                dy = p_path->y[j] - p_path->y[i];
                p_path->yaw[i] = atan2(dy, dx);
                p_path->ds[i] = hypot(dy, dx);
            }

        }
    }


    p_path->x.erase(p_path->x.begin()+j, p_path->x.end());
    p_path->y.erase(p_path->y.begin()+j, p_path->y.end());
    p_path->yaw.erase(p_path->yaw.begin()+j, p_path->yaw.end());
    p_path->ds.erase(p_path->ds.begin()+j, p_path->ds.end());
    p_path->c.erase(p_path->c.begin()+j, p_path->c.end());
//    PRINT_DEBUG("j = {:d}, p_path->x.size() = {:d}, p_path->yaw[{:d}] = {:.3f}, p_path->yaw[{:d}] = {:.3f}",
//                j, p_path->x.size(), j-2, p_path->yaw[j-2], j-1, p_path->yaw[j-1]);
    if(p_path->x.size() < 2)
    {
        //PRINT_DEBUG("path size = {:d}", path_size);
        return false;
    }
//    for(int j = 0; j < p_path->x.size() - 1; ++j)
//    {
//        dx = p_path->x[j+1] - p_path->x[j];
//        dy = p_path->y[j+1] - p_path->y[j];
//        p_path->yaw.push_back(atan2(dy, dx));
//        p_path->ds.push_back(hypot(dy, dx));
//        //PRINT_DEBUG("dx, dy = {:.3f}, {:.3f}, atan2(dy, dx+ = {:.1f}", dx, dy, radian_to_degree(atan2(dy, dx)));
//    }
    p_path->yaw.back() = p_path->yaw[j-2];
    p_path->ds.back() = p_path->ds[j-2];
    for(int j = 0; j < p_path->yaw.size() - 1; ++j)
    {
        dyaw = p_path->yaw[j + 1] - p_path->yaw[j];
        dyaw = constraint_angle_r(dyaw, -M_PI, M_PI);
        c = dyaw / p_path->ds[j];
        p_path->c.push_back(c);
        if(fabs(c) > max_curvature)
        {
            PRINT_WARN("curvature({:.3f}) > max_curvature({:.3f})", fabs(c), max_curvature);
            return false;
        }
    }
    return true;
}

bool FrenetOptimalPlanner::select_best_path(std::vector<Path> *p_path_list, CubicSpline2D *p_csp, Path *p_best_path)
{
    auto cmp = [](const Path* a, const Path* b)
    {
        return a->cf > b->cf;
    };

    std::priority_queue<Path*, std::vector<Path*>, decltype(cmp)> qu(cmp);
    int path_list_size = p_path_list->size();
    //PRINT_DEBUG("path list size = {:d}", path_list_size);
    for(int i = 0; i < path_list_size; ++i)
    {
        qu.push((Path*)&(*p_path_list)[i]);
    }
    bool is_exist_valid_path = false;
    const uint8_t obstacles_cost = (uint8_t)(mp_map->obstacles_cost() - 2);
    while (!qu.empty() && !is_exist_valid_path)
    {
        Path *p_path = qu.top();
        qu.pop();
        //PRINT_DEBUG("cf = {:.3f}", p_path->cf);
        if(!calc_global_path(p_path, p_csp))
        {
            //PRINT_WARN("calc global path failed");
            continue;
        }
        else
        {
            bool is_detect_collision = false;
            //check collision
            int path_size = p_path->x.size();

            for(int i = 0; i < path_size; ++i)
            {
                //Pose<float> pose(p_path->x[i], p_path->y[i], p_path->yaw[i]);
                //if(mp_map->map_collision_detect(pose))
                if(mp_map->cost(p_path->x[i], p_path->y[i]) > obstacles_cost)
                {
                    //PRINT_WARN("collision pos: {:.3f}, {:.3f}", p_path->x[i], p_path->y[i]);
                    is_detect_collision = true;
                    break;
                }
            }
            if(is_detect_collision)
            {
                continue;
            }
            *p_best_path = *p_path;
            is_exist_valid_path = true;
        }
    }
    //PRINT_DEBUG("best path cost = {:.3f}", p_best_path->cf);
    return is_exist_valid_path;
}

}
