#include "frenet_polynomial_planner.h"
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <cmath>
#include <fstream>
#include <queue>
#include "order_5th_polynomial.h"
#include "common/json.hpp"
#include "common/common.h"
#include "common/print.h"
#include "common/time_keeper.h"
#include "frenet_cartesian.h"
#include "frenet_polynomial_planner/cubic_spline_2d.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "controller_to_ros.h"
#include "modules/map/map_base.h"

namespace bz_robot
{
namespace frenet_polynomial_planner
{
FrenetPolynomialPlanner::FrenetPolynomialPlanner()
{
    default_config();
}


void FrenetPolynomialPlanner::default_config()
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
    m_goal_explore_range = -3;
    m_longitude_step = 0.5;
}



bool FrenetPolynomialPlanner::import_config(const char *config_file)
{
    try
    {
        PRINT_INFO("read config: {}", config_file);
        std::ifstream i(config_file);
        if(i)
        {
            nlohmann::json j;
            i >> j;
            FLOAT_T max_road_width = j["max_road_width"];
            FLOAT_T road_width_step = j["road_width_step"];
//            FLOAT_T min_prediction_time = j["min_prediction_time"];
//            FLOAT_T max_prediction_time = j["max_prediction_time"];
//            FLOAT_T time_step = j["time_step"];
//            FLOAT_T target_speed = j["target_speed"];
//            FLOAT_T speed_step = j["speed_step"];
//            FLOAT_T speed_step_number = j["speed_step_number"];
            FLOAT_T max_curvature = j["max_curvature"];
            FLOAT_T kd = j["kd"];
            FLOAT_T kj = j["kj"];
            FLOAT_T kt = j["kt"];
            FLOAT_T klat = j["klat"];
            FLOAT_T klon = j["klon"];

            m_max_road_width = max_road_width;
            m_road_width_step = road_width_step;
//            m_min_prediction_time = min_prediction_time;
//            m_max_prediction_time = max_prediction_time;
//            m_dt = time_step;
//            m_target_speed = target_speed;
//            m_speed_step = speed_step;
//            m_speed_step_number = speed_step_number;
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
        PRINT_ERROR("exception: {}", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("un expexted occured");
    }

    return false;
}

bool FrenetPolynomialPlanner::plan(std::shared_ptr<MapBase> p_map, Pose<FLOAT_T> start_pose,
                                Pose<FLOAT_T> goal_pose, std::vector<Pose<FLOAT_T> > *p_path)
{
    RECORD_TIME();
    const std::vector<Pose<FLOAT_T> > &global_path = m_ref_path;
    mp_map = p_map;

    int global_path_size = global_path.size();
    std::cout<<"=========================="<<std::endl;
//    std::cout<<"global_path_size="<<global_path_size<<std::endl;
//    for(int i = 0; i < global_path_size; ++i)
//    {
//      std::cout<<global_path[i].position.x<<std::endl;
//    }
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
    FLOAT_T mx = 0;
    FLOAT_T my = 0;
//    uint32_t mx = 0;
//    uint32_t my = 0;
    //将全局路径由世界坐标系转化到地图坐标系
    for(int i = 0; i < global_path_size; ++i)
    {
        if(mp_map->world_to_map(global_path[i].position.x, global_path[i].position.y, &mx, &my))
        {
            if(i == 0)
            {
                m_global_path_x.emplace_back((FLOAT_T)mx);
                m_global_path_y.emplace_back((FLOAT_T)my);
                m_global_path_heading_angle.emplace_back(global_path[i].heading_angle);
            }
            else
            {
                if(round(mx * 100) != round(m_global_path_x.back() * 100) ||
                   round(my * 100) != round(m_global_path_y.back() * 100))
                {
                    m_global_path_x.emplace_back((FLOAT_T)mx);
                    m_global_path_y.emplace_back((FLOAT_T)my);
                    m_global_path_heading_angle.emplace_back(global_path[i].heading_angle);
                }
            }
        }
        else
        {
            PRINT_ERROR("world pose({:.3f}, {:.3f}) to map error!", global_path[i].position.x, global_path[i].position.y);
        }
    }
    //将机器人位置转化到地图坐标系
    if(mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &mx, &my))
    {
        m_map_pose_start = Pose<FLOAT_T>(mx, my, start_pose.heading_angle);
    }
    else
    {
        return false;
    }
    //将全局路径上的点转化到frenet坐标系，d都为0
    CubicSpline2D csp = CubicSpline2D(m_global_path_x, m_global_path_y);
    for(int i = 0; i < m_global_path_x.size(); ++i)
    {
        std::vector<FLOAT_T> frenet_s_position = get_frenet(m_global_path_x[i], m_global_path_y[i], m_global_path_heading_angle[i],
                                                    m_global_path_x, m_global_path_y);
        m_global_path_frenet_s.emplace_back(frenet_s_position[0]);
        m_global_path_frenet_d.emplace_back(frenet_s_position[1]);
        //PRINT_DEBUG("global path({:.3f}, {:.3f}) => sd({:.3f}, {:.3f})", m_global_path_x[i], m_global_path_y[i], frenet_s_position[0], frenet_s_position[1]);
    }

    std::vector<FLOAT_T> frenet_start_pose = get_frenet(m_map_pose_start.position.x, m_map_pose_start.position.y,
                                                       m_map_pose_start.heading_angle,
                                                       m_global_path_x, m_global_path_y);
    if(frenet_start_pose.size() < 2)
    {
        return false;
    }
//    PRINT_DEBUG("cur pose({:.3f}, {:.3f}, {:.3f}) => sd({:.3f}, {:.3f})",
//                m_map_pose_start.position.x, m_map_pose_start.position.y,
//                radian_to_degree(m_map_pose_start.heading_angle),
//                frenet_start_pose[0], frenet_start_pose[1]);

    //FLOAT_T s0 = 0.0;  //current course position
    const FLOAT_T s_start_position = frenet_start_pose[0];
    //FLOAT_T c_d = 0.0;  //current lateral position [m]
    const FLOAT_T d_start_position = frenet_start_pose[1];

    //FrenetPolynomialPlanner::Path path = calc_frenet_path(s_start_position, d_start_position, &csp);
    //PRINT_INFO("total path {:d}", path_list.size());
    FrenetPolynomialPlanner::Path best_path;
    bool flag_is_plan_success = calc_frenet_path2(s_start_position, d_start_position, &csp, &best_path);
    if(flag_is_plan_success)
    {
        p_path->clear();
        p_path->resize(best_path.x.size());
        FLOAT_T wx = 0;
        FLOAT_T wy = 0;
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

    return flag_is_plan_success;
}

//bool FrenetPolynomialPlanner::calc_frenet_path(const FLOAT_T &s_start_position,
//                                                const FLOAT_T &d_start_position,
//                                                CubicSpline2D *p_csp,
//                                                FrenetPolynomialPlanner::Path *p_path)
//{
//    auto cmp = [](const Path& a, const Path& b)
//    {
//        return a.score > b.score;
//    };
//    std::priority_queue<Path, std::vector<Path>, decltype(cmp)> path_list(cmp);

//    FLOAT_T x = 0;
//    FLOAT_T y = 0;
//    FLOAT_T yaw = 0;
//    FLOAT_T fx = 0;
//    FLOAT_T fy = 0;
//    FLOAT_T dx = 0;
//    FLOAT_T dy = 0;
////    FLOAT_T dyaw = 0;
////    FLOAT_T c = 0;
////    FLOAT_T lateral_deviation, lateral_velocity, lateral_acceleration, lateral_jerk;
////    FLOAT_T longitudinal_acceleration, longitudinal_jerk;
//    //横向动作规划
//    const FLOAT_T max_road_width = m_max_road_width / mp_map->resolution();
//    const FLOAT_T road_width_step = m_road_width_step / mp_map->resolution();

//    //const FLOAT_T min_goal_s = std::max(0.0f, m_global_path_frenet_s.back() + m_goal_explore_range / mp_map->resolution());
//    const FLOAT_T max_goal_s = m_global_path_frenet_s.back();
//    const FLOAT_T min_goal_s = m_global_path_frenet_s.back() * 0.4;
//    const FLOAT_T s_step = m_longitude_step / mp_map->resolution();
//    const FLOAT_T t = 10.0f;
//    const FLOAT_T max_curvature = m_max_curvature * mp_map->resolution();
//    const uint32_t pose_size = uint32_t(m_global_path_frenet_s.back() - s_start_position) + 1;
//    bool flag_is_path_vaild = true;
//    bool flag_is_path_contains_obstacles = true;
//    std::vector<Order5thPolynomial> d_path_list;
//    std::vector<Order5thPolynomial> s_path_list;
//    std::vector<std::vector<FLOAT_T>> d_path_pose_list;
//    std::vector<std::vector<FLOAT_T>> s_path_pose_list;

//    const int d_index_range = 3 + int(max_road_width * 0.5 / road_width_step);
//    //for(FLOAT_T di  = -max_road_width; di <= max_road_width; di += road_width_step)
//    for(int d_index  = 0; d_index <= d_index_range; ++d_index)
//    {
//        //const FLOAT_T d_v = di == 0 ? 0 :  di / fabs(di);

//        FLOAT_T di = ((d_index + 1) / 2) * road_width_step * pow(-1, d_index);
//        di = std::max(FLOAT_T(-max_road_width * 0.5), di);
//        di = std::min(FLOAT_T(max_road_width * 0.5), di);
//        const FLOAT_T d_v = 0;
//        //PRINT_DEBUG("d path {}, {}, {}, {}, {}, {}, {}", d_start_position, 0, 0.0f, di, 0, 0, pose_size);
//        Order5thPolynomial d_path = Order5thPolynomial(d_start_position, 0, 0.0f, di, 0, 0, pose_size-1);
//        d_path_list.emplace_back(d_path);
//        std::vector<FLOAT_T> d_path_pose;
//        d_path_pose.resize(pose_size);
//        //PRINT_DEBUG("di = {}", di);
//        for(uint32_t i = 0; i != pose_size; ++i)
//        {
//            d_path_pose[i] = d_path.calc_point(i);
//            //PRINT_DEBUG("d[{}]: {:.2f}", i, d_path_pose[i]);
//        }
//        //printf("\n\n");
//        d_path_pose_list.emplace_back(d_path_pose);
//    }

//    for(FLOAT_T si  = max_goal_s; si > min_goal_s; si -= s_step)
//    {
//        //PRINT_DEBUG("s path {}, {}, {}, {}, {}, {}, {}", s_start_position, 1.0f, 0.0f, si, 1.0f, 0.0f, pose_size);
//        Order5thPolynomial s_path = Order5thPolynomial(s_start_position, 1.0f, 0.0f, si, 1.0f, 0.0f, pose_size-1);
//        s_path_list.emplace_back(s_path);
//        std::vector<FLOAT_T> s_path_pose;
//        s_path_pose.resize(pose_size);
//        for(uint32_t i = 0; i != pose_size; ++i)
//        {
//            s_path_pose[i] = s_path.calc_point(i);
//            //PRINT_DEBUG("s[{}]: {:.2f}", i, s_path_pose[i]);
//        }
//        //printf("\n\n");
//        s_path_pose_list.emplace_back(s_path_pose);
//    }

//    for(int s_index = 0; s_index < s_path_pose_list.size(); ++s_index)
//    {
//        for(int d_index = 0; d_index < d_path_pose_list.size(); ++d_index)
//        {
//            //std::vector<FLOAT_T> &s_path_pose = s_path_pose_list[s_index];
//            flag_is_path_vaild = true;
//            Path path = Path();
//            path.resize(pose_size);
//            path.s = s_path_pose_list[s_index];
//            path.d = d_path_pose_list[d_index];
//            //PRINT_INFO("pose_size = {}", pose_size);
//            int curvature_outrange_counter = 0;
//            for(int i = 0; i < pose_size; ++i)
//            {
//                FLOAT_T s = path.s[i];
//                FLOAT_T d = path.d[i];
//                if(m_is_use_csp)
//                {
//                    x = p_csp->calc_x(s);
//                    y = p_csp->calc_y(s);
//                    if(std::isnan(x) || std::isnan(y))
//                    {
//                        PRINT_ERROR("x = {}, y = {}", x, y);
//                        break;
//                    }
//                    yaw = p_csp->calc_yaw(s);
//                    fx = x + d * cos(yaw + M_PI_2);
//                    fy = y + d * sin(yaw + M_PI_2);
//                    path.x[i] = fx;
//                    path.y[i] = fy;

//                }
//                else
//                {
//                    std::vector<FLOAT_T> pos = get_xy(s, d, m_global_path_frenet_s, m_global_path_x, m_global_path_y);
//                    path.x[i] = pos[0];
//                    path.y[i] = pos[1];

//                    #if 0
//                    if(s > m_global_path_frenet_s.back())
//                    {
//                        ++j;
//                        break;
//                    }
//                    #endif
//                }

//                if(i > 0)
//                {
//                    int j = i-1;
//                    dx = path.x[i] - path.x[j];
//                    dy = path.y[i] - path.y[j];
//                    path.yaw[i] = atan2(dy, dx);
//                    path.ds[i] = hypot(dy, dx);
//                    //PRINT_ERROR("dx = {}, dy = {}", dx, dy);
//                    //PRINT_INFO("[{}] yaw = {}", i, radian_to_degree(path.yaw[i]));
//                    if(i < pose_size - 1)
//                    {

//                        const FLOAT_T temp_dx0 = path.x[i] - path.x[j];
//                        const FLOAT_T temp_dy0 = path.y[i] - path.y[j];
//                        const FLOAT_T temp_dx1 = path.x[i+1] - path.x[i];
//                        const FLOAT_T temp_dy1 = path.y[i+1] - path.y[i];
//                        const FLOAT_T temp_dt0 = hypot(temp_dx0, temp_dy0);
//                        const FLOAT_T temp_dt1 = hypot(temp_dx1, temp_dy1);
//                        const FLOAT_T temp_sum_1 = 1.0f / (temp_dt0 + temp_dt1);
//                        const FLOAT_T temp_dx = temp_dt1 * temp_sum_1 + temp_dx0 + temp_dt0 * temp_sum_1 * temp_dx1;
//                        const FLOAT_T temp_dy = temp_dt1 * temp_sum_1 + temp_dy0 + temp_dt0 * temp_sum_1 * temp_dy1;
//                        path.yaw[i] = atan2(temp_dy, temp_dx);
//                        //p_path->ds[i] = temp_dt0;
//                        //PRINT_INFO("[{}] yaw2 = {}", i, radian_to_degree(path.yaw[i]));

//                        //calc
//                        Pose<FLOAT_T> p1;
//                        p1.position.x = path.x[j];
//                        p1.position.y = path.y[j];
//                        p1.heading_angle = path.yaw[j];
//                        Pose<FLOAT_T> p2;
//                        p2.position.x = path.x[i];
//                        p2.position.y = path.y[i];
//                        p2.heading_angle = path.yaw[i];
//                        FLOAT_T curvature = calc_curvature(p1, p2);
//                        if(curvature > max_curvature)
//                        {
//                            ++curvature_outrange_counter;
//                        }
//                        else
//                        {
//                            curvature_outrange_counter = 0;
//                        }
//                        if(curvature_outrange_counter > 3)
//                        {
//                            //skip cur curvature
//                            flag_is_path_vaild = false;
//                            PRINT_ERROR("curvature = {} > {}, it's too big", curvature, max_curvature);
//                            continue;
//                        }
//                    }
//                    else
//                    {
//        //                int i = j-1;
//        //                dx = p_path->x[j] - p_path->x[i];
//        //                dy = p_path->y[j] - p_path->y[i];
//                        path.yaw[i] = atan2(dy, dx);
//                        path.ds[i] = hypot(dy, dx);
//                    }
//                }
//            }

//            //if(flag_is_path_vaild && calc_path_score(&path))
//            if(calc_path_score(&path))
//            {
//                path_list.push(path);
//                flag_is_path_contains_obstacles = false;
//            }
//        }
//        if(!path_list.empty())
//        {
//            *p_path = path_list.top();
////            while(!path_list.empty())
////            {
////                PRINT_DEBUG("path score = {}", path_list.top().score);
////                path_list.pop();
////            }
//            //路径可视化
//            int size = path_list.size();
//            std::vector<Path> ros_path_list;
//            ros_path_list.resize(size);
//            for(int i=0;i<size;i++)
//            {
//              ros_path_list[i] = std::move(path_list.top());
//              path_list.pop();
//            }
//            send_path_list_to_ros(ros_path_list);
//            return true;
//        }
//    }

//    if(flag_is_path_contains_obstacles)
//    {
//        PRINT_ERROR("all path contains obstacles {}", flag_is_path_contains_obstacles);
//    }

//    return false;
//}



//2 是先产生各种线路，后面是与全局路径平行
bool FrenetPolynomialPlanner::calc_frenet_path2(const FLOAT_T &s_start_position,
                                                const FLOAT_T &d_start_position,
                                                CubicSpline2D *p_csp,
                                                FrenetPolynomialPlanner::Path *p_path)
{
    auto cmp = [](const Path& a, const Path& b)
    {
        return a.score > b.score;
    };
    std::priority_queue<Path, std::vector<Path>, decltype(cmp)> path_list(cmp);

    FLOAT_T x = 0;
    FLOAT_T y = 0;
    FLOAT_T yaw = 0;
    FLOAT_T fx = 0;
    FLOAT_T fy = 0;
    FLOAT_T dx = 0;
    FLOAT_T dy = 0;
//    FLOAT_T dyaw = 0;
//    FLOAT_T c = 0;
//    FLOAT_T lateral_deviation, lateral_velocity, lateral_acceleration, lateral_jerk;
//    FLOAT_T longitudinal_acceleration, longitudinal_jerk;
    //转化为地图坐标系的最大路径宽度，以及宽度步长，主要用于设置预备局部路径的条数阈值
    const FLOAT_T max_road_width = m_max_road_width / mp_map->resolution();
    const FLOAT_T road_width_step = m_road_width_step / mp_map->resolution();
    //传递过来的全局路径长度
    const FLOAT_T max_goal_s = m_global_path_frenet_s.back();
    //允许的最大曲率？？？？？？？？？？？？？？？
    const FLOAT_T max_curvature = m_max_curvature * mp_map->resolution();
    //delta_s决定了从当前d位置到达di需要经过的s长度，越长越平滑
    FLOAT_T delta_s = std::min(FLOAT_T(3.5 / mp_map->resolution()),
                               FLOAT_T(m_global_path_frenet_s.back() - s_start_position)); //unit m / map resolution
    const uint32_t pose_size = uint32_t(m_global_path_frenet_s.back() - s_start_position) + 1;
    uint32_t delta_pose_size = uint32_t(delta_s) + 1;

    bool flag_is_path_vaild = true;
    bool flag_is_path_contains_obstacles = true;
    std::vector<Order5thPolynomial> d_path_list;
    std::vector<std::vector<FLOAT_T>> d_path_pose_list;
    //决定了局部路径的条数，在d方向上以全局路径为轴对称分布
    const int d_index_range = 3 + int(max_road_width * 0.5 / road_width_step);
    for(int d_index  = 0; d_index <= d_index_range; ++d_index)
    {
        FLOAT_T di = (int((d_index + 1) / 2)) * road_width_step * pow(-1, d_index);
        di = std::max(FLOAT_T(-max_road_width * 0.5), di);
        di = std::min(FLOAT_T(max_road_width * 0.5), di);
        Order5thPolynomial d_path = Order5thPolynomial(d_start_position, 0, 0.0f, di, 0, 0, delta_pose_size-1);
        d_path_list.emplace_back(d_path);
        std::vector<FLOAT_T> d_path_pose;
        d_path_pose.resize(pose_size);
        //0-delta_s需要五次多项式插值得到，delta_s-m_global_path_frenet_s.back()都取di
        for(uint32_t i = 0; i < delta_pose_size; ++i)
        {
            d_path_pose[i] = d_path.calc_point(i);
        }
        for(uint32_t i = delta_pose_size; i < pose_size; ++i)
        {
            d_path_pose[i] = di;
        }
        d_path_pose_list.emplace_back(d_path_pose);
    }
    //在s方向上进行5次多项式插值，最后在0-delta_s需要五次多项式插值得到，delta_s-m_global_path_frenet_s.back()等距离取值
    Order5thPolynomial standard_s_path = Order5thPolynomial(s_start_position, 1.0f, 0.0f, s_start_position + delta_s,
                                                            1.0f, 0.0f, delta_pose_size-1);
    std::vector<FLOAT_T> standard_s_path_pose;
    standard_s_path_pose.resize(pose_size);
    for(uint32_t i = 0; i < delta_pose_size; ++i)
    {
        standard_s_path_pose[i] = standard_s_path.calc_point(i);
    }
    for(uint32_t i = delta_pose_size; i < pose_size; ++i)
    {
        standard_s_path_pose[i] = s_start_position + delta_s +
                (max_goal_s - s_start_position - delta_s) / (pose_size - delta_pose_size) * (i - delta_pose_size + 1);
        //PRINT_DEBUG("index [{}], s {}", i, standard_s_path_pose[i]);
    }
    standard_s_path_pose.back() = max_goal_s;
    //依次遍历距离全局路径不同横向距离的路径，计算车头偏向角
    for(int d_index = 0; d_index < d_path_pose_list.size(); ++d_index)
    {
        //std::vector<FLOAT_T> &s_path_pose = s_path_pose_list[s_index];
        flag_is_path_vaild = true;
        Path path = Path();
        path.resize(pose_size);
        path.s = standard_s_path_pose;
        path.d = d_path_pose_list[d_index];

        //PRINT_INFO("pose_size = {}", pose_size);
        //连续三个点曲率不满足要求，放弃当前路径
        int curvature_outrange_counter = 0;
        for(int i = 0; i < pose_size; ++i)
        {
            FLOAT_T s = path.s[i];
            FLOAT_T d = path.d[i];

            std::vector<FLOAT_T> pos = get_xy(s, d, m_global_path_frenet_s, m_global_path_x, m_global_path_y);
            path.x[i] = pos[0];
            path.y[i] = pos[1];

            #if 0
            if(s > m_global_path_frenet_s.back())
            {
                ++j;
                break;
            }
            #endif


            if(i > 0)
            {
                int j = i-1;
                dx = path.x[i] - path.x[j];
                dy = path.y[i] - path.y[j];
                path.yaw[i] = atan2(dy, dx);
                path.ds[i] = hypot(dy, dx);
                //PRINT_ERROR("dx = {}, dy = {}", dx, dy);
                //PRINT_INFO("[{}] yaw = {}", i, radian_to_degree(path.yaw[i]));
                if(i < pose_size - 1)
                {
                    //temp_dx0.temp_dy0表示当前点相对于上一个点的x,y变化
                    //temp_dx1.temp_dy1表示下一个点相对于当前点的x,y变化
                    //temp_dt0，temp_dt1为两段线段的长度
                    //
                    const FLOAT_T temp_dx0 = path.x[i] - path.x[j];
                    const FLOAT_T temp_dy0 = path.y[i] - path.y[j];
                    const FLOAT_T temp_dx1 = path.x[i+1] - path.x[i];
                    const FLOAT_T temp_dy1 = path.y[i+1] - path.y[i];
                    const FLOAT_T temp_dt0 = hypot(temp_dx0, temp_dy0);
                    const FLOAT_T temp_dt1 = hypot(temp_dx1, temp_dy1);
                    const FLOAT_T temp_sum_1 = 1.0f / (temp_dt0 + temp_dt1);
//                    const FLOAT_T temp_dx = temp_dt1 * temp_sum_1 + temp_dx0 + temp_dt0 * temp_sum_1 * temp_dx1;
//                    const FLOAT_T temp_dy = temp_dt1 * temp_sum_1 + temp_dy0 + temp_dt0 * temp_sum_1 * temp_dy1;
                    //todo 原本的计算方法为上面注释的两行
                    const FLOAT_T temp_dx = temp_dt1 * temp_sum_1 * temp_dx0 + temp_dt0 * temp_sum_1 * temp_dx1;
                    const FLOAT_T temp_dy = temp_dt1 * temp_sum_1 * temp_dy0 + temp_dt0 * temp_sum_1 * temp_dy1;
                    path.yaw[i] = atan2(temp_dy, temp_dx);
                    //std::cout<<"yaw1="<<atan2(temp_dy0, temp_dx0)<<"/yaw2="<<atan2(temp_dy, temp_dx)<<"/yaw3="<<atan2(temp_dy1, temp_dx1)<<std::endl;
                    //p_path->ds[i] = temp_dt0;
                    //PRINT_INFO("[{}] yaw2 = {}", i, radian_to_degree(path.yaw[i]));

                    //calc
                    Pose<FLOAT_T> p1;
                    p1.position.x = path.x[j];
                    p1.position.y = path.y[j];
                    p1.heading_angle = path.yaw[j];
                    Pose<FLOAT_T> p2;
                    p2.position.x = path.x[i];
                    p2.position.y = path.y[i];
                    p2.heading_angle = path.yaw[i];
                    FLOAT_T curvature = calc_curvature(p1, p2);
                    if(curvature > max_curvature)
                    {
                        ++curvature_outrange_counter;
                    }
                    else
                    {
                        curvature_outrange_counter = 0;
                    }
                    if(curvature_outrange_counter > 3)
                    {
                        //skip cur curvature
                        flag_is_path_vaild = false;
                        PRINT_ERROR("curvature = {} > {}, it's too big", curvature, max_curvature);
                        continue;
                    }
                }
                else
                {
    //                int i = j-1;
    //                dx = p_path->x[j] - p_path->x[i];
    //                dy = p_path->y[j] - p_path->y[i];
                    path.yaw[i] = atan2(dy, dx);
                    path.ds[i] = hypot(dy, dx);
                }
            }
        }

        //if(flag_is_path_vaild && calc_path_score(&path))
        if(calc_path_score(&path))
        {
            path_list.push(path);
            flag_is_path_contains_obstacles = false;
        }
        else {
            path_list.push(path);
        }
    }
    if(!path_list.empty())
    {
        *p_path = path_list.top();
        if(p_path->d.back() != 0)
        {
            PRINT_ERROR("best path di = {}, cost = {}", p_path->d.back(), p_path->score);
        }
//            while(!path_list.empty())
//            {
//                PRINT_DEBUG("path score = {}", path_list.top().score);
//                path_list.pop();
//            }

          //路径可视化
          int size = path_list.size();
          std::vector<Path> ros_path_list;
          ros_path_list.resize(size);
          for(int i=0;i<size;i++)
          {
            ros_path_list[i] = std::move(path_list.top());
            path_list.pop();
          }
          send_path_list_to_ros(ros_path_list);

        return true;
    }

    if(flag_is_path_contains_obstacles)
    {
        PRINT_ERROR("all path contains obstacles {}", flag_is_path_contains_obstacles);
    }

    return false;
}


bool FrenetPolynomialPlanner::calc_path_score(FrenetPolynomialPlanner::Path * p_path)
{
    FrenetPolynomialPlanner::Path& path = *p_path;
    const FLOAT_T cost_d_path = 1.0;
    const FLOAT_T cost_yaw = 1.0 / M_PI;
    const FLOAT_T cost_obstacles = 1.0;
    const int obstacles_cost = mp_map->obstacles_cost() - 2;
    const int path_size = path.x.size();
    FLOAT_T cost = 0;
    float cost1;
    float cost2;
    float cost3;
    for(int i = 0; i < path_size; ++i)
    {
        cost += fabs(path.d[i]) * cost_d_path;
        cost1 = cost;
        //std::cout<<"距离为："<<path.d[i]<<std::endl;
        if(i > 0)
        {
            FLOAT_T yaw = path.yaw[i] - path.yaw[i-1];
            yaw = constraint_angle_r(yaw, -M_PI, M_PI);
            cost += fabs(yaw) * cost_yaw;
            //std::cout<<"角度差为："<<yaw<<std::endl;
        }
        cost2 = cost - cost1;
        uint32_t mx = path.x[i];
        uint32_t my = path.y[i];
        //uint8_t map_cost = mp_map->cost(mx, my);
        int map_cost = mp_map->cost(mx, my);
        //std::cout<<"网格代价为："<<map_cost<<std::endl;
        if(map_cost > obstacles_cost)
        {
            //PRINT_DEBUG("({}, {}) cost: {}", mx, my, map_cost);
            return false;
        }
        else
        {
            cost += mp_map->cost(path.x[i], path.y[i]) * cost_obstacles;
        }
        cost3 = cost - cost1 - cost2;
        //std::cout<<"距离路径的代价："<<cost1<<"、角度变化的代价："<<cost2<<"、关于障碍物的代价："<<cost3<<std::endl;
    }

    p_path->score = cost;
    //PRINT_DEBUG("di: {}, score: {}", path.d.back(), cost);
    return true;
}

FLOAT_T FrenetPolynomialPlanner::calc_curvature(const Pose<FLOAT_T> &p1, const Pose<FLOAT_T> &p2)
{
    float yaw1 = p1.heading_angle;
    yaw1 = constraint_angle_r(yaw1);
    if(0.0 == yaw1)
    {
        yaw1 = 0.001;
    }
    float k1 = -1.0 / tan(yaw1);
    float a1 = -k1;
    float b1 = 1.0;
    float c1 = -b1 * p1.position.y - a1 * p1.position.x;

    float yaw2 = p2.heading_angle;
    yaw2 = constraint_angle_r(yaw2);
    if(0.0 == yaw2)
    {
        yaw2 = 0.001;
    }
    float k2 = -1.0 / tan(yaw2);
    float a2 = -k2;
    float b2 = 1.0;
    float c2 = -b2 * p2.position.y
                - a2 * p2.position.x;

    float r_x = (c1*b2-c2*b1)/(a2*b1-a1*b2);
    float r_y = (a2*c1-a1*c2)/(a1*b2-a2*b1);

    float r = hypot(r_x - p1.position.x,
                     r_y - p1.position.y);
    if(round(yaw1 * 1000) == round(yaw2 * 1000))
    {
        r = 1000000;
    }
    r = std::min(r, 100000.0f);
    return 1.0f / r;
}


void FrenetPolynomialPlanner::send_path_list_to_ros(const std::vector<Path>& path_list)
{
    //路径可视化
    int size = path_list.size();
    std::vector<nav_msgs::Path> visual_paths;
    visual_paths.clear();
    for(int i=0;i<size;i++)
    {
      Path pi = path_list[i];
      nav_msgs::Path    path;
      for(int j=0;j<pi.x.size();j++)
      {
        geometry_msgs::PoseStamped p;
        float x = 0.0;
        float y = 0.0;
        mp_map->map_to_world(pi.x.at(j),pi.y.at(j),&x,&y);
        p.pose.position.x    = x;
        p.pose.position.y    = y;
        p.pose.position.z    = 0.0;
        p.pose.orientation.w =  1.0;
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = 0.0;
        path.poses.push_back(p);
      }
      //path_list.pop();
      visual_paths.push_back(path);
    }
    bz_robot::MessageToRos::GetInstance()->VisuallocalPath(visual_paths);
}

}
}
