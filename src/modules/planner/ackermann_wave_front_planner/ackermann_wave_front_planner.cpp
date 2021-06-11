#include "ackermann_wave_front_planner.h"
#include <stdio.h>
#include <cassert>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <exception>
#include <algorithm>
#include <mcheck.h>
#include <queue>
#include <boost/heap/binomial_heap.hpp>
#include <boost/heap/priority_queue.hpp>
#include <list>
#include "common/print.h"
#include "common/time_keeper.h"
#include "common/common.h"
#include "common/json.hpp"
#include "common/pnm_image.h"
//#include "path_smoother/path_smoother.h"

namespace bz_robot
{
namespace ackermann_wave_front_planner
{

static const float INF = std::numeric_limits<float>::infinity();

WaveFrontPlanner::WaveFrontPlanner():
    m_resolution(0)
{
    //m_path_smooth.set_smooth_times(500);
    //即使不在构造函数内部进行初始化，列表初始化也会对类类型成员进行初始化
    m_fmm_planner = fmm_planner::FMMPlanner();
    default_config();
}

bool WaveFrontPlanner::plan(std::shared_ptr<MapBase> p_map, Pose<float> pose_start, Pose<float> pose_goal,
                            std::vector<Pose<float> > *p_path)
{
    mp_map = p_map;
    m_start_pose = pose_start;
    m_goal_pose = pose_goal;
    bool is_find_path = false;
    p_path->clear();
    uint32_t map_start_pose_x;
    uint32_t map_start_pose_y;
    uint32_t map_goal_pose_x;
    uint32_t map_goal_pose_y;
    //判断点是否在地图范围内
    if(!mp_map->world_to_map(pose_start.position.x, pose_start.position.y, &map_start_pose_x, &map_start_pose_y))
    {
        return false;
    }
    //存放以指定车头偏向角以及前轮转角移动单位步长后，各运动参量dx,dy,dthete的变化情况，分辨率目前为0.5m,这是为了后期查表使用
    cache_look_up_table(mp_map->resolution());

    //更新fmm cost map
    fmm_explore(pose_start, pose_goal, p_path);
    //是否为快速模式
    if(! m_fast_mode)
    {
        is_find_path = sorted_wave_front3(pose_start, pose_goal, p_path);
    }
    else
    {
        is_find_path = wave_front(pose_start, pose_goal, p_path);
    }
    //p_path->back() = (pose_goal);
    //p_path->emplace_back(pose_goal);
    if(is_find_path)
    {
//        if(m_is_enable_smooth)
//        {
//            m_path_smooth.set_smooth_times(m_smooth_times);
//            std::vector<Pose<float>> smooth_path;
//            smooth_path = *p_path;
//            for(int i = 0; i < smooth_path.size(); ++i)
//            {
//                mp_map->world_to_map(smooth_path[i].position.x, smooth_path[i].position.y,
//                                     &smooth_path[i].position.x, &smooth_path[i].position.y);
//            }
//            m_path_smooth.update_map(mp_map);
//            m_path_smooth.smoothPath(smooth_path);
//            smooth_path = m_path_smooth.getSmoothedPath();
//            for(int i = 0; i < smooth_path.size(); ++i)
//            {
//                mp_map->map_to_world(smooth_path[i].position.x, smooth_path[i].position.y,
//                                     &smooth_path[i].position.x, &smooth_path[i].position.y);
//        //        PRINT_DEBUG("[{:d}] ({:.3f}, {:.3f}) ==  ({:.3f}, {:.3f})", i,
//        //                    (*p_path)[i].position.x, (*p_path)[i].position.y,
//        //                    smooth_path[i].position.x, smooth_path[i].position.y);
//            }
//            *p_path = smooth_path;
//        }
//        p_path->front() = pose_start;
//        p_path->back() = pose_goal;
//#if 1
//        if(p_path->size() >= 6)
//        {
//            std::vector<PathOptimizationNS::State> result_path;
//            std::vector<PathOptimizationNS::State> reference_path;
//            PathOptimizationNS::State state_start(pose_start.position.x, pose_start.position.y, pose_start.heading_angle);
//            PathOptimizationNS::State state_goal(pose_goal.position.x, pose_goal.position.y, pose_goal.heading_angle);
//            PathOptimizationNS::PathOptimizer path_optimizer(state_start, state_goal, mp_map);
//            reference_path.resize(p_path->size());
//            for(int i = 0; i != reference_path.size(); ++i)
//            {
//                reference_path[i].x = (*p_path)[i].position.x;
//                reference_path[i].y = (*p_path)[i].position.y;
//                reference_path[i].z = (*p_path)[i].heading_angle;
//            }
//            if (path_optimizer.solve(reference_path, &result_path))
//            {
//                std::cout << "ok!" << std::endl;
//                // Test solveWithoutSmoothing:
//                //path_optimizer.solveWithoutSmoothing(result_path, &result_path);
//                p_path->resize(result_path.size());
//                for(int i = 0; i != result_path.size(); ++i)
//                {
//                    (*p_path)[i].position.x = result_path[i].x;
//                    (*p_path)[i].position.y = result_path[i].y;
//                    (*p_path)[i].heading_angle = result_path[i].z;
//                }
//            }
//        }
//#endif
        const int path_size = (*p_path).size();
        VectorX2<float> start_pose_error = pose_start.position - (*p_path)[0].position;
        VectorX2<float> goal_pose_error = pose_goal.position - (*p_path)[path_size-1].position;

        for(int i = 0; i < path_size; ++i)
        {
            if(i < path_size/2)
            {
                (*p_path)[i].position += start_pose_error * (1.0 * (path_size/2 - i) / (path_size/2));
            }
            else
            {
                (*p_path)[i].position += goal_pose_error * (1.0 * (i - (path_size/2)) / (path_size - 1 - path_size/2));
            }
        }
        p_path->back() = pose_goal;

    }

    return is_find_path;
}

bool WaveFrontPlanner::import_config(const char *config_file)
{
    try
    {
        PRINT_INFO("read config: {}", config_file);
        std::ifstream i(config_file);
        if(i)
        {
            nlohmann::json j;
            i >> j;
            float angle_step = j["ANGLE_STEP"];
            float distance_cost_ratio= j["DISTANCE_COST_RATIO"];
            int step_size = j["STEP_SIZE"];
            float step_cost_ratio = j["STEP_COST_RATIO"];
            float delta_heading_cost_ratio = j["DELTA_HEADING_COST_RATIO"];
            float map_cost_ratio = j["MAP_COST_RATIO"];
            float akermann_wheel_base = j["AKERMANN_WHEEL_BASE"];
            float akermann_max_steer_angle = j["AKERMANN_MAX_STEER_ANGLE"];
            bool fast_mode = j["FAST_MODE"];
            bool is_enable_smooth = j["IS_ENABLE_SMOOTH"];
            uint32_t smooth_times = j["SMOOTH_TIMES"];
            if(step_size <= 0)
            {
                exit(-1);
            }
            m_angle_step = degree_to_radian(angle_step);
            m_distance_cost_ratio = int(distance_cost_ratio);
            m_step_size = step_size;
            m_step_cost_ratio = int(step_cost_ratio);
            m_delta_heading_cost_ratio = delta_heading_cost_ratio * radian_to_degree(1.0);
            m_map_cost_ratio = map_cost_ratio;
            m_akermann_wheel_base = akermann_wheel_base;
            m_akermann_max_steer_angle = degree_to_radian(akermann_max_steer_angle);
            m_fast_mode = fast_mode;
            m_is_enable_smooth = is_enable_smooth;
            m_smooth_times = smooth_times;
            return true;
        }
        else
        {
            PRINT_ERROR("can't read config files from: {}\n", config_file);
        }
    }
    catch(std::exception& e )
    {
        PRINT_ERROR("{}\n", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("un expexted occured\n");
    }

    return false;
}

void WaveFrontPlanner::default_config()
{
    //m_angle_step = degree_to_radian(10);
    //map resulution 0.5~1 -> 10
    m_angle_step = degree_to_radian(10);
    m_distance_cost_ratio = 10;
    m_step_size = 1;
    m_step_cost_ratio = 5;
    m_delta_heading_cost_ratio = 0.05 * radian_to_degree(1.0);
    m_map_cost_ratio = 0.4;
    m_akermann_wheel_base = 0.64;
    m_akermann_max_steer_angle = degree_to_radian(20);
    m_fast_mode = false;
}


bool WaveFrontPlanner::wave_front_explore(const Pose<float> &start_pose, const Pose<float> &goal_pose, std::vector<Pose<float> > *p_path)
{
    RECORD_TIME();
    const int8_t obstacle_value = mp_map->obstacles_cost()-1;
    uint32_t map_w = mp_map->size_in_cells_x();
    uint32_t map_h = mp_map->size_in_cells_y();
    m_wave_front_look_up_table.resize(map_w);

    for(uint32_t i = 0; i < map_w; ++i)
    {
        m_wave_front_look_up_table[i].assign(map_h, -1);
    }

    uint32_t map_pose_x;
    uint32_t map_pose_y;
    mp_map->world_to_map(goal_pose.position.x, goal_pose.position.y, &map_pose_x, &map_pose_y);
    m_wave_front_look_up_table[map_pose_x][map_pose_y] = 0;
    //std::map<uint32_t, int> wave;
    //uint32_t index = mp_map->index(map_pose_x, map_pose_y);
    //int cost = 0;
    //wave[index] = cost;
    const static int pos_array[4][2] = {{1, 0}, {0, 1},  {-1, 0}, {0, -1}};
    //static int pos_array[8][2] = {{1, 0}, {0, 1},  {-1, 0}, {0, -1}, {1,1}, {-1, 1}, {-1, -1}, {1, -1}};
    const uint32_t array_size = sizeof(pos_array) / sizeof(pos_array[0]);
    std::queue<VectorX2<int>> qu;
    qu.push(VectorX2<int>(map_pose_x, map_pose_y));
    //uint32_t it = 0;
    uint32_t map_pose_start_x;
    uint32_t map_pose_start_y;
    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_start_x, &map_pose_start_y);
    bool is_find_path = false;
    while(!qu.empty())
    {

        VectorX2<int> map_pose = qu.front();
        qu.pop();
        //explore wave
        for(int i = 0; i < array_size; ++i)
        {
            //++it;
            VectorX2<int> next_map_pose;
            next_map_pose.x = map_pose.x + pos_array[i][0];
            next_map_pose.y = map_pose.y + pos_array[i][1];

            if(next_map_pose.x < map_w && next_map_pose.y < map_h)
            {
                if(m_wave_front_look_up_table[next_map_pose.x][next_map_pose.y] == -1)
                {
                    int map_cost = (int)mp_map->cost((uint32_t)next_map_pose.x, (uint32_t)next_map_pose.y);

                    if(map_cost < obstacle_value)
                    {
                        qu.push(next_map_pose);
                        m_wave_front_look_up_table[next_map_pose.x][next_map_pose.y] = m_wave_front_look_up_table[map_pose.x][map_pose.y] + 1;
                    }
                }
            }
        }
    }
    return is_find_path;
}

bool WaveFrontPlanner::fmm_explore(const Pose<float> &start_pose, const Pose<float> &goal_pose, std::vector<Pose<float> > *p_path)
{
    //todo 更新fmm地图
    m_fmm_planner.fmm_explore(mp_map, start_pose, goal_pose);
    m_fmm_look_up_table = std::move(m_fmm_planner.costmap());
}

bool WaveFrontPlanner::sorted_wave_front(Pose<float> start_pose, Pose<float> goal_pose, std::vector<Pose<float> > *p_path)
{
    //RECORD_TIME();
    // cost x y heading
    typedef std::array<uint32_t, 4> Cell;
    //std::swap(start_pose, goal_pose);

    //const int8_t obstacle_value = mp_map->obstacles_cost()-1;
    m_wave_map.clear();
    m_wave_front_prev_node.clear();
    m_map_w = mp_map->size_in_cells_x();
    m_map_h = mp_map->size_in_cells_y();
    //获得机器人x,y，theta坐标对应的栅格编号，round 表示四舍五入，这里根据角度分辨率获得机器人方向角对应的编号
    uint32_t map_pose_x;
    uint32_t map_pose_y;
    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_x, &map_pose_y);
    start_pose.heading_angle = constraint_angle_r(start_pose.heading_angle);
    uint32_t header_index = round(start_pose.heading_angle / m_angle_step);
    //根据特定的函数获得
    uint64_t index = pose_to_index(map_pose_x, map_pose_y, header_index);
    m_wave_map[index][0] = 0;
    m_wave_map[index][1] = map_pose_x;
    m_wave_map[index][2] = map_pose_y;
    m_wave_map[index][3] = start_pose.heading_angle;

    auto cmp = [this](const Cell& a, const Cell& b)
    {
        const uint32_t ratio = m_distance_cost_ratio;
        //wave_front
//        return a[I_COST] + m_wave_front_look_up_table[a[I_X]][a[I_Y]] * ratio
//               > b[I_COST] + m_wave_front_look_up_table[b[I_X]][b[I_Y]] * ratio;
        // fmm
        return a[I_COST] + m_fmm_look_up_table[a[I_X]][a[I_Y]] * ratio
               > b[I_COST] + m_fmm_look_up_table[b[I_X]][b[I_Y]] * ratio;
    };

    //std::priority_queue<Pose<float>, std::vector<Pose<float>>, decltype(cmp)> qu(cmp);
    std::priority_queue<Cell, std::vector<Cell>, decltype(cmp)> qu(cmp);
    //boost::heap::binomial_heap<Pose<float>, boost::heap::compare<decltype(cmp)>> qu(cmp);
    //boost::heap::priority_queue<Pose<float>, boost::heap::compare<decltype(cmp)>> qu(cmp);
    //std::queue<VectorX2<int>> qu;
    PRINT_INFO("start node: {:d}, {:d}, {:d}, %lu", map_pose_x, map_pose_y, header_index, index);
    qu.push(Cell{0, map_pose_x, map_pose_y, header_index});

    uint32_t it = 0;
    uint32_t map_pose_goal_x;
    uint32_t map_pose_goal_y;
    goal_pose.heading_angle = constraint_angle_r(goal_pose.heading_angle) ;
    uint32_t map_pose_goal_heading_index = round(goal_pose.heading_angle / m_angle_step);
    mp_map->world_to_map(goal_pose.position.x, goal_pose.position.y, &map_pose_goal_x, &map_pose_goal_y);
    uint64_t goal_index = pose_to_index(Pose<int>(map_pose_goal_x, map_pose_goal_y, map_pose_goal_heading_index));
    PRINT_DEBUG("goal({}, {}, {}) : {}\n\n", map_pose_goal_x, map_pose_goal_y, map_pose_goal_heading_index, goal_index);

    bool is_find_path = false;
    const uint32_t look_up_table_size = m_look_up_table[0].size();

    uint32_t max_qu_size = qu.size();
    uint32_t re_sort_counter = 0;
    uint32_t skip_it = 0;
    p_path->clear();
    while(!qu.empty())
    {

        if(it % 100 == 0)
        {
            if(m_flag_stop)
            {
                m_flag_stop = false;
                return false;
            }
        }
        Cell cur_cell = qu.top();
        qu.pop();
        //Pose<int> map_pose_index(round(map_pose.position.x), round(map_pose.position.y), round(map_pose.heading_angle/m_angle_step));
        //Pose<int> map_pose_index(int(map_pose.position.x), int(map_pose.position.y), round(map_pose.heading_angle/m_angle_step));
        uint64_t cur_index = pose_to_index(cur_cell[I_X], cur_cell[I_Y], cur_cell[I_HEADING]);
        if(cur_index == goal_index)
        {
            PRINT_INFO("find akermann path\n\n\n");
//            PRINT_DEBUG("(%f, %f, %f) %f", map_pose.position.x, map_pose.position.y, map_pose.heading_angle,
//                        m_wave_front_costmap[map_pose.position.x][map_pose.position.y][map_pose.heading_angle][0]);
            is_find_path = true;
            generate_path(cur_index, p_path);
            PRINT_INFO("re_sort_counter = {}, skip {:d}, max_qu_size = {}, it = {}", re_sort_counter, skip_it, max_qu_size, it);
            return is_find_path;
        }

        //explore wave
        if(cur_cell[I_COST] > (uint32_t)m_wave_map[cur_index][I_COST])
        {
            ++skip_it;
            continue;
        }

        for(int i = 0; i < look_up_table_size; ++i)
        {
            ++it;
            float next_x = m_wave_map[cur_index][I_X] + m_look_up_table[cur_cell[I_HEADING]][i].delta_x;
            float next_y = m_wave_map[cur_index][I_Y] + m_look_up_table[cur_cell[I_HEADING]][i].delta_y;
            float next_heading = constraint_angle_r(m_wave_map[cur_index][I_HEADING] + m_look_up_table[cur_cell[I_HEADING]][i].delta_heading);
            //float steer = m_look_up_table[map_pose.heading_angle][i].steer_angle;
            //Pose<float> next_map_pose(next_x, next_y, next_heading);
            //Pose<int> next_map_pose_index;
//            next_map_pose_index.position.x = round(next_x);
//            next_map_pose_index.position.y = round(next_y);
//            next_map_pose_index.heading_angle = round(next_heading / m_angle_step);
            Cell next_cell;
            next_cell[I_X] = uint32_t(next_x);
            next_cell[I_Y] = uint32_t(next_y);
            next_cell[I_HEADING] = (uint32_t)round(next_heading / m_angle_step);

            if(next_cell[I_X] < m_map_w && next_cell[I_Y] < m_map_h)
            {
                //if(m_wave_front_costmap[next_map_pose.x][next_map_pose.y] == -1)
                {
                    int map_cost = (int)mp_map->cost(next_cell[I_X], next_cell[I_Y]);
                    //wavefront
                    //if(m_wave_front_look_up_table[next_cell[I_X]][next_cell[I_Y]] != -1)
                    //fmm
                    if(m_fmm_look_up_table[next_cell[I_X]][next_cell[I_Y]] != INF)
                    {
                        //float steer_angle = fabs(m_look_up_table[map_pose.heading_angle][i].steer_angle);
                        uint64_t next_index = pose_to_index(next_cell[I_X], next_cell[I_Y], next_cell[I_HEADING]);

                        uint32_t cost = (uint32_t)m_wave_map[cur_index][I_COST] + m_step_cost_ratio;
                        //cost += int(fabs(steer_angle) * steer_angle_ratio);
                        cost += uint32_t(m_delta_heading_cost_ratio * fabs(m_look_up_table[cur_cell[I_HEADING]][i].delta_heading));
                        cost += (uint32_t)(map_cost * m_map_cost_ratio);
                        //cost = m_wave_map[cur_index][0] + 10;

                        if(m_wave_map.find(next_index) == m_wave_map.end())
                        {
                            m_wave_map[next_index][I_COST] = cost;
                            m_wave_map[next_index][1] = next_x;
                            m_wave_map[next_index][2] = next_y;
                            m_wave_map[next_index][3] = next_heading;
                            //PRINT_DEBUG("(%f %f, %f) %lu", next_x, next_y, next_heading * 180 * M_1_PI, next_index);
                            next_cell[I_COST] = cost;
                            qu.push(next_cell);
                            m_wave_front_prev_node[next_index] = cur_index;
                        }
                        else
                        {
                            if(cost < m_wave_map[next_index][0])
                            {
                                //PRINT_DEBUG("update cost\n\n\n\n");
                                m_wave_map[next_index][I_COST] = cost;
                                m_wave_map[next_index][1] = next_x;
                                m_wave_map[next_index][2] = next_y;
                                m_wave_map[next_index][3] = next_heading;
                                //PRINT_DEBUG("(%f %f, %f) %lu", next_x, next_y, next_heading * 180 * M_1_PI, next_index);
                                next_cell[I_COST] = cost;
                                qu.push(next_cell);
                                m_wave_front_prev_node[next_index] = cur_index;
                                ++re_sort_counter;
                            }
                        }

                        //for test
                        if(qu.size() > max_qu_size)
                        {
                            max_qu_size = qu.size();
                        }
                    }
                }
            }
        }
    }
    PRINT_INFO("re_sort_counter = {}, max_qu_size = {}, it = {}", re_sort_counter, max_qu_size, it);
    PRINT_ERROR("can't find path !!!\n\n\n");
    return is_find_path;
}


void WaveFrontPlanner::cache_look_up_table(const float resolution)
{
    RECORD_TIME();
    if(m_resolution != resolution)
    {
        m_resolution = resolution;
        const float wheel_base = m_akermann_wheel_base / m_resolution;
        const float max_steer_angle = m_akermann_max_steer_angle;//大小为20°
        const float angle_step = m_angle_step;//大小为10°
        bool allow_backword = false;
        //floor(x)小于等于 x,且与 x 最接近的整数
        //heading_size存放所有可能的车头角度信息，大小为36
        int heading_size = 2 * floor(M_PI / angle_step);
        //steer_size存放所有可能的偏转角信息，大小为5
        int steer_size = 2 * floor(max_steer_angle / angle_step) + 1;
        const float steer_angle_step = angle_step;
        //int steer_size = 3;
        //const float steer_angle_step = m_akermann_max_steer_angle;
        m_look_up_table.resize(heading_size);

        m_header_size = heading_size;
        m_steer_angle_size = steer_size;
        //按照10度的角分辨率划分机器人车头角度以及前轮转角，存放不同车头角度以及前轮偏向角下的（dx, dy, delta_heading, steer_angle）
        for(int heading_index = 0; heading_index < heading_size; ++heading_index)
        {
            float heading = heading_index * angle_step;
            for(int steer_index = 0; steer_index < steer_size; ++steer_index)
            {
                #if 0
                float steer_angle = (steer_size/2 - steer_index) * angle_step;
                #else
                float steer_angle = 0;
                if(steer_index%2 == 0)
                {
                    steer_angle = (steer_index / 2) * steer_angle_step;
                }
                else
                {
                    steer_angle = -(int)((steer_index+1) / 2) * steer_angle_step;
                }
                #endif
                float step = 100;
                //float step = 1.414;
                float beta;
                float dx;
                float dy;
                float delta_heading;
//                printf("heading = %f, steer_angle = %f\n",
//                       heading * 180 * M_1_PI, steer_angle * 180 * M_1_PI);
                //v > 0
                beta = atan(0.5 * tan(steer_angle));
                dx = step * cos(beta + heading);
                dy = step * sin(beta + heading);
                //当fabs(dx) = fabs(dy)时step=1.414
                if(fabs(dx) > fabs(dy))
                {
                    step = step / fabs(dx);
                    dx = round(step * cos(beta + heading));
                    dy = step * sin(beta + heading);
                }
                else
                {
                    step = step / fabs(dy);
                    dx = step * cos(beta + heading);
                    dy = round(step * sin(beta + heading));
                }
                delta_heading = step * tan(steer_angle) / wheel_base;
                {
                  //std::cout<<"车头偏向角："<<steer_angle<<"  机器人偏向角变化："<<delta_heading<<"  dx："<<dx<<"  dy："<<dy<<std::endl;
                  m_look_up_table[heading_index].push_back(MoveStep(dx, dy, delta_heading, steer_angle));
                }
                //v < 0
                if(allow_backword)
                {
                    beta = atan(0.5 * tan(steer_angle));
                    dx = step * cos(beta + heading);
                    dy = step * sin(beta + heading);

                    if(fabs(dx) > fabs(dy))
                    {
                        step = step / fabs(dx);
                        step = step * -1.0;
                        dx = round(step * cos(beta + heading));
                        dy = step * sin(beta + heading);
                    }
                    else
                    {
                        step = step / fabs(dy);
                        step = step * -1.0;
                        dx = step * cos(beta + heading);
                        dy = round(step * sin(beta + heading));
                    }
                    delta_heading = step * tan(steer_angle) / wheel_base;
                    {
                        m_look_up_table[heading_index].push_back(MoveStep(dx, dy, delta_heading, steer_angle));
                    }
                }
            }
        }
    }
}

inline uint64_t WaveFrontPlanner::pose_to_index(const uint32_t x, const uint32_t y, const uint32_t heading_index)
{
    return (uint64_t)heading_index + (uint64_t)y * m_header_size + (uint64_t)x * m_header_size * m_map_h;
}

inline uint64_t WaveFrontPlanner::pose_to_index(const Pose<int> pose)
{
    return (uint64_t)pose.heading_angle + (uint64_t)pose.position.y * m_header_size + (uint64_t)pose.position.x * m_header_size * m_map_h;
}

inline Pose<int> WaveFrontPlanner::index_to_pose(uint64_t index)
{
    Pose<int> pose;
    pose.position.x = index / (m_header_size * m_map_h);
    pose.position.y = (index - pose.position.x * (m_header_size * m_map_h)) / m_header_size;
    pose.heading_angle = index - pose.position.x * (m_header_size * m_map_h) - pose.position.y * m_header_size;
    return pose;
}

void WaveFrontPlanner::generate_path(uint64_t index, std::vector<Pose<float> > *p_path)
{
    //std::vector<Pose<float> > map_path;
    while (1)
    {
        //Pose<int> cur_pose = index_to_pose(index);
        //map_path.emplace_back(Pose<float>(m_wave_map[index][1], m_wave_map[index][2],m_wave_map[index][3]));
        Pose<float> pose;
        mp_map->map_to_world(m_wave_map[index][1], m_wave_map[index][2], &pose.position.x, &pose.position.y);
        pose.heading_angle = m_wave_map[index][3];
        p_path->emplace_back(pose);
        //cur_pose = index_to_pose(index);
        //PRINT_INFO("{:.3f}, {:.3f} => {:.3f}, {:.3f}", m_wave_map[index][1], m_wave_map[index][2], pose.position.x, pose.position.y);
        index = m_wave_front_prev_node[index];
        if(m_wave_front_prev_node.find(index) == m_wave_front_prev_node.end())
        {
            break;
        }
    }
    //p_path->emplace_back(m_start_pose);
    reverse(p_path->begin(), p_path->end());
    const int path_size = p_path->size();

    VectorX2<float> start_pose_error = m_start_pose.position - (*p_path)[0].position;
    VectorX2<float> goal_pose_error = m_goal_pose.position - (*p_path)[path_size-1].position;

    for(int i = 0; i < path_size; ++i)
    {
        if(i < path_size/2)
        {
            (*p_path)[i].position += start_pose_error * (1.0 * (path_size/2 - i) / (path_size/2));
        }
        else
        {
            (*p_path)[i].position += goal_pose_error * (1.0 * (i - (path_size/2)) / (path_size - 1 - path_size/2));
        }
    }
    p_path->front() = m_start_pose;
    p_path->back() = m_goal_pose;
}

bool WaveFrontPlanner::wave_front(Pose<float> start_pose, Pose<float> goal_pose, std::vector<Pose<float> > *p_path)
{
    RECORD_TIME();
    // cost x y heading
    typedef std::array<uint32_t, 4> Cell;
    //std::swap(start_pose, goal_pose);

    //获取地图量
    m_wave_map.clear();
    m_wave_front_prev_node.clear();
    m_map_w = mp_map->size_in_cells_x();
    m_map_h = mp_map->size_in_cells_y();
    //计算栅格地图的位置角度编号，以及组合编号
    uint32_t map_pose_x;
    uint32_t map_pose_y;
    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_x, &map_pose_y);
    start_pose.heading_angle = constraint_angle_r(start_pose.heading_angle);
    uint32_t header_index = round(start_pose.heading_angle / m_angle_step);
    uint64_t index = pose_to_index(map_pose_x, map_pose_y, header_index);
    m_wave_map[index][0] = 0;
    m_wave_map[index][1] = map_pose_x;
    m_wave_map[index][2] = map_pose_y;
    m_wave_map[index][3] = start_pose.heading_angle;
    //匿名比较函数
    auto cmp = [this](const Cell& a, const Cell& b)
    {
        const uint32_t ratio = m_distance_cost_ratio;
        return a[I_COST] + m_wave_front_look_up_table[a[I_X]][a[I_Y]] * ratio
               > b[I_COST] + m_wave_front_look_up_table[b[I_X]][b[I_Y]] * ratio;
    };

    //std::priority_queue<Pose<float>, std::vector<Pose<float>>, decltype(cmp)> qu(cmp);
    std::priority_queue<Cell, std::vector<Cell>, decltype(cmp)> qu(cmp);
    //boost::heap::binomial_heap<Pose<float>, boost::heap::compare<decltype(cmp)>> qu(cmp);
    //boost::heap::priority_queue<Pose<float>, boost::heap::compare<decltype(cmp)>> qu(cmp);
    //std::queue<VectorX2<int>> qu;
    PRINT_INFO("start node: {:d}, {:d}, {:d}, {}", map_pose_x, map_pose_y, header_index, index);
    qu.push(Cell{0, map_pose_x, map_pose_y, header_index});

    uint32_t it = 0;
    uint32_t map_pose_goal_x;
    uint32_t map_pose_goal_y;
    goal_pose.heading_angle = constraint_angle_r(goal_pose.heading_angle) ;
    uint32_t map_pose_goal_heading_index = round(goal_pose.heading_angle / m_angle_step);
    mp_map->world_to_map(goal_pose.position.x, goal_pose.position.y, &map_pose_goal_x, &map_pose_goal_y);
    uint64_t goal_index = pose_to_index(Pose<int>(map_pose_goal_x, map_pose_goal_y, map_pose_goal_heading_index));
    PRINT_DEBUG("goal({}, {}, {}) : {}\n\n", map_pose_goal_x, map_pose_goal_y, map_pose_goal_heading_index, goal_index);

    bool is_find_path = false;
    //const uint32_t look_up_table_size = m_look_up_table[0].size();

    uint32_t max_qu_size = qu.size();
    uint32_t re_sort_counter = 0;

    while(!qu.empty())
    {

        Cell cur_cell = qu.top();
        qu.pop();
        //Pose<int> map_pose_index(round(map_pose.position.x), round(map_pose.position.y), round(map_pose.heading_angle/m_angle_step));
        //Pose<int> map_pose_index(int(map_pose.position.x), int(map_pose.position.y), round(map_pose.heading_angle/m_angle_step));
        uint64_t cur_index = pose_to_index(cur_cell[1], cur_cell[2], cur_cell[3]);
        if(cur_index == goal_index)
        {
            PRINT_INFO("find akermann path\n\n\n");
            //            PRINT_DEBUG("(%f, %f, %f) %f", map_pose.position.x, map_pose.position.y, map_pose.heading_angle,
            //                        m_wave_front_costmap[map_pose.position.x][map_pose.position.y][map_pose.heading_angle][0]);
            is_find_path = true;
            generate_path(cur_index, p_path);
            PRINT_INFO("re_sort_counter = {}, max_qu_size = {}, it = {}", re_sort_counter, max_qu_size, it);
            return is_find_path;
        }

        //explore wave
        if(cur_cell[I_COST] > (uint32_t)m_wave_map[cur_index][I_COST])
        {
            continue;
        }

        //for(int i = 0; i < m_look_up_table[cur_cell[I_HEADING]].size(); ++i)
        for(int x = 0; x < 3; ++x)
        {
            int i = 0;
            if(x == 0)
            {
                i = 0;
            }
            else if(x == 1)
            {
                i = m_look_up_table[cur_cell[I_HEADING]].size()-1;
            }
            else if(x == 2)
            {
                i = m_look_up_table[cur_cell[I_HEADING]].size()-2;
            }
            ++it;
            float next_x = m_wave_map[cur_index][I_X] + m_look_up_table[cur_cell[I_HEADING]][i].delta_x;
            float next_y = m_wave_map[cur_index][I_Y] + m_look_up_table[cur_cell[I_HEADING]][i].delta_y;
            float next_heading = constraint_angle_r(m_wave_map[cur_index][I_HEADING] + m_look_up_table[cur_cell[I_HEADING]][i].delta_heading);
            //float steer = m_look_up_table[map_pose.heading_angle][i].steer_angle;
            //Pose<float> next_map_pose(next_x, next_y, next_heading);
            //Pose<int> next_map_pose_index;
            //            next_map_pose_index.position.x = round(next_x);
            //            next_map_pose_index.position.y = round(next_y);
            //            next_map_pose_index.heading_angle = round(next_heading / m_angle_step);
            Cell next_cell;
            next_cell[I_X] = uint32_t(next_x);
            next_cell[I_Y] = uint32_t(next_y);
            next_cell[I_HEADING] = (uint32_t)round(next_heading / m_angle_step);

            if(next_cell[I_X] < m_map_w && next_cell[I_Y] < m_map_h)
            {
                //if(m_wave_front_costmap[next_map_pose.x][next_map_pose.y] == -1)
                {
                    int map_cost = (int)mp_map->cost(next_cell[I_X], next_cell[I_Y]);

                    if(m_wave_front_look_up_table[next_cell[I_X]][next_cell[I_Y]] != -1)
//                    if(!check_obstacles(cur_cell[I_X], cur_cell[I_Y],
//                                         (int)(next_cell[I_X] - cur_cell[I_X]),
//                                         (int)(next_cell[I_Y] - cur_cell[I_Y])))
                    {
                        //float steer_angle = fabs(m_look_up_table[map_pose.heading_angle][i].steer_angle);
                        uint64_t next_index = pose_to_index(next_cell[I_X], next_cell[I_Y], next_cell[I_HEADING]);

                        uint32_t cost = (uint32_t)m_wave_map[cur_index][0] + m_step_cost_ratio;
                        //cost += int(fabs(steer_angle) * steer_angle_ratio);
                        cost += uint32_t(m_delta_heading_cost_ratio * fabs(m_look_up_table[cur_cell[I_HEADING]][i].delta_heading));
                        cost += (uint32_t)(map_cost * m_map_cost_ratio);
                        //cost = m_wave_map[cur_index][0] + 10;
                        next_cell[I_COST] = cost;
                        if(m_wave_map.find(next_index) == m_wave_map.end())
                        {
                            m_wave_map[next_index][0] = cost;
                            m_wave_map[next_index][1] = next_x;
                            m_wave_map[next_index][2] = next_y;
                            m_wave_map[next_index][3] = next_heading;
                            //PRINT_DEBUG("(%f %f, %f) %lu", next_x, next_y, next_heading * 180 * M_1_PI, next_index);
                            qu.push(next_cell);
                            m_wave_front_prev_node[next_index] = cur_index;
                        }

                        //for test
                        if(qu.size() > max_qu_size)
                        {
                            max_qu_size = qu.size();
                        }
                    }
                }
            }
        }
    }
    PRINT_INFO("re_sort_counter = {}, max_qu_size = {}, it = {}", re_sort_counter, max_qu_size, it);
    PRINT_ERROR("can't find path !!!\n\n\n");
    return is_find_path;
}


bool WaveFrontPlanner::check_obstacles(const uint32_t x, const uint32_t y, const int dx, const int dy)
{
    bool is_exit_obstacles = false;
    const uint32_t n = std::max(dx, dy);
    const float n_1 = 1.0 / n;
    for(int i = 0; i < dx; ++i)
    {
        uint32_t x_pos = round(x + i * dx * n_1);
        uint32_t y_pos = round(y + i * dy * n_1);
        if(m_wave_front_look_up_table[x_pos][y_pos] == -1)
        {
            is_exit_obstacles = true;
            break;
        }
    }
    return is_exit_obstacles;
}

bool WaveFrontPlanner::sorted_wave_front2(Pose<float> start_pose, Pose<float> goal_pose, std::vector<Pose<float> > *p_path)
{
    RECORD_TIME();
    if(!find_path(start_pose, goal_pose, p_path))
    {
        PRINT_ERROR("can't find path");
        return false;
    }
    inflate_path(*p_path);
    p_path->clear();

    // cost x y heading
    typedef std::array<uint32_t, 4> Cell;
    //std::swap(start_pose, goal_pose);

    const uint8_t obstacle_value = mp_map->obstacles_cost()-1;
    m_wave_map.clear();
    m_wave_front_prev_node.clear();
    m_map_w = mp_map->size_in_cells_x();
    m_map_h = mp_map->size_in_cells_y();

    uint32_t map_pose_x;
    uint32_t map_pose_y;
    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_x, &map_pose_y);
    start_pose.heading_angle = constraint_angle_r(start_pose.heading_angle);
    uint32_t header_index = round(start_pose.heading_angle / m_angle_step);
    uint64_t index = pose_to_index(map_pose_x, map_pose_y, header_index);
    m_wave_map[index][0] = 0;
    m_wave_map[index][1] = map_pose_x;
    m_wave_map[index][2] = map_pose_y;
    m_wave_map[index][3] = start_pose.heading_angle;

    auto cmp = [this](const Cell& a, const Cell& b)
    {
        const uint32_t ratio = m_distance_cost_ratio;
        return a[I_COST] + m_wave_front_look_up_table[a[I_X]][a[I_Y]] * ratio
               > b[I_COST] + m_wave_front_look_up_table[b[I_X]][b[I_Y]] * ratio;
    };

    //std::priority_queue<Pose<float>, std::vector<Pose<float>>, decltype(cmp)> qu(cmp);
    std::priority_queue<Cell, std::vector<Cell>, decltype(cmp)> qu(cmp);
    //boost::heap::binomial_heap<Pose<float>, boost::heap::compare<decltype(cmp)>> qu(cmp);
    //boost::heap::priority_queue<Pose<float>, boost::heap::compare<decltype(cmp)>> qu(cmp);
    //std::queue<VectorX2<int>> qu;
    PRINT_INFO("start node: {:d}, {:d}, {:d}, {}", map_pose_x, map_pose_y, header_index, index);
    qu.push(Cell{0, map_pose_x, map_pose_y, header_index});

    uint32_t it = 0;
    uint32_t map_pose_goal_x;
    uint32_t map_pose_goal_y;
    goal_pose.heading_angle = constraint_angle_r(goal_pose.heading_angle) ;
    uint32_t map_pose_goal_heading_index = round(goal_pose.heading_angle / m_angle_step);
    mp_map->world_to_map(goal_pose.position.x, goal_pose.position.y, &map_pose_goal_x, &map_pose_goal_y);
    uint64_t goal_index = pose_to_index(Pose<int>(map_pose_goal_x, map_pose_goal_y, map_pose_goal_heading_index));
    PRINT_DEBUG("goal({}, {}, {}) : {}\n\n", map_pose_goal_x, map_pose_goal_y, map_pose_goal_heading_index, goal_index);

    bool is_find_path = false;
    const uint32_t look_up_table_size = m_look_up_table[0].size();

    uint32_t max_qu_size = qu.size();
    uint32_t re_sort_counter = 0;
    uint32_t skip_it = 0;
    while(!qu.empty())
    {

        Cell cur_cell = qu.top();
        qu.pop();
        //Pose<int> map_pose_index(round(map_pose.position.x), round(map_pose.position.y), round(map_pose.heading_angle/m_angle_step));
        //Pose<int> map_pose_index(int(map_pose.position.x), int(map_pose.position.y), round(map_pose.heading_angle/m_angle_step));
        uint64_t cur_index = pose_to_index(cur_cell[I_X], cur_cell[I_Y], cur_cell[I_HEADING]);
        if(cur_index == goal_index)
        {
            PRINT_INFO("find akermann path\n\n\n");
            //            PRINT_DEBUG("(%f, %f, %f) %f", map_pose.position.x, map_pose.position.y, map_pose.heading_angle,
            //                        m_wave_front_costmap[map_pose.position.x][map_pose.position.y][map_pose.heading_angle][0]);
            is_find_path = true;
            generate_path(cur_index, p_path);
            PRINT_INFO("re_sort_counter = {}, skip {:d}, max_qu_size = {}, it = {}", re_sort_counter, skip_it, max_qu_size, it);
            return is_find_path;
        }

        //explore wave
        if(cur_cell[I_COST] > (uint32_t)m_wave_map[cur_index][I_COST])
        {
            ++skip_it;
            continue;
        }

        //for(int i = 0; i < look_up_table_size; ++i)
        for(int x = 0; x < 3; ++x)
        {
            int i = 0;
            if(x == 1)
            {
                i = look_up_table_size-1;
            }
            if(x == 2)
            {
                i = look_up_table_size-2;
            }

            ++it;
            float next_x = m_wave_map[cur_index][I_X] + m_look_up_table[cur_cell[I_HEADING]][i].delta_x;
            float next_y = m_wave_map[cur_index][I_Y] + m_look_up_table[cur_cell[I_HEADING]][i].delta_y;
            float next_heading = constraint_angle_r(m_wave_map[cur_index][I_HEADING] + m_look_up_table[cur_cell[I_HEADING]][i].delta_heading);

            Cell next_cell;
            next_cell[I_X] = uint32_t(next_x);
            next_cell[I_Y] = uint32_t(next_y);
            next_cell[I_HEADING] = (uint32_t)round(next_heading / m_angle_step);

            if(next_cell[I_X] < m_map_w && next_cell[I_Y] < m_map_h)
            {
                //if(m_wave_front_costmap[next_map_pose.x][next_map_pose.y] == -1)
                {
                    int map_cost = (int)mp_map->cost(next_cell[I_X], next_cell[I_Y]);

                    if(m_ref_path_costmap[next_cell[I_X]][next_cell[I_Y]] < obstacle_value)
                    {
                        //float steer_angle = fabs(m_look_up_table[map_pose.heading_angle][i].steer_angle);
                        uint64_t next_index = pose_to_index(next_cell[I_X], next_cell[I_Y], next_cell[I_HEADING]);

                        uint32_t cost = (uint32_t)m_wave_map[cur_index][I_COST] + m_step_cost_ratio;
                        //cost += int(fabs(steer_angle) * steer_angle_ratio);
                        cost += uint32_t(m_delta_heading_cost_ratio * fabs(m_look_up_table[cur_cell[I_HEADING]][i].delta_heading));
                        cost += (uint32_t)(map_cost * m_map_cost_ratio);
                        //cost += m_ref_path_costmap[next_cell[I_X]][next_cell[I_Y]];
                        cost += m_ref_path_3dcostmap[next_cell[I_X]][next_cell[I_Y]][next_cell[I_HEADING]] * 10;
                        //cost = m_wave_map[cur_index][0] + 10;

                        if(m_wave_map.find(next_index) == m_wave_map.end())
                        {
                            m_wave_map[next_index][I_COST] = cost;
                            m_wave_map[next_index][1] = next_x;
                            m_wave_map[next_index][2] = next_y;
                            m_wave_map[next_index][3] = next_heading;
                            //PRINT_DEBUG("(%f %f, %f) %lu", next_x, next_y, next_heading * 180 * M_1_PI, next_index);
                            next_cell[I_COST] = cost;
                            qu.push(next_cell);
                            m_wave_front_prev_node[next_index] = cur_index;
                        }
//                        else
//                        {
//                            if(cost < m_wave_map[next_index][0])
//                            {
//                                //PRINT_DEBUG("update cost\n\n\n\n");
//                                m_wave_map[next_index][I_COST] = cost;
//                                m_wave_map[next_index][1] = next_x;
//                                m_wave_map[next_index][2] = next_y;
//                                m_wave_map[next_index][3] = next_heading;
//                                //PRINT_DEBUG("(%f %f, %f) %lu", next_x, next_y, next_heading * 180 * M_1_PI, next_index);
//                                next_cell[I_COST] = cost;
//                                qu.push(next_cell);
//                                m_wave_front_prev_node[next_index] = cur_index;
//                                ++re_sort_counter;
//                            }
//                        }

                        //for test
                        if(qu.size() > max_qu_size)
                        {
                            max_qu_size = qu.size();
                        }
                    }
                }
            }
        }
    }
    PRINT_INFO("re_sort_counter = {}, max_qu_size = {}, it = {}", re_sort_counter, max_qu_size, it);
    PRINT_ERROR("can't find path !!!\n\n\n");
    return is_find_path;
}

//全方向运动的路径，不考虑转弯半径
bool WaveFrontPlanner::find_path(Pose<float> start_pose, Pose<float> goal_pose, std::vector<Pose<float> > *p_path)
{
    RECORD_TIME();
    const int8_t obstacle_value = mp_map->obstacles_cost()-1;
    uint32_t map_w = mp_map->size_in_cells_x();
    uint32_t map_h = mp_map->size_in_cells_y();
    std::vector<std::vector<int>> wave_front_costmap;
    wave_front_costmap.resize(map_w);

    for(uint32_t i = 0; i < map_w; ++i)
    {
        wave_front_costmap[i].assign(map_h, -1);
    }

    uint32_t map_pose_x;
    uint32_t map_pose_y;
    mp_map->world_to_map(goal_pose.position.x, goal_pose.position.y, &map_pose_x, &map_pose_y);
    wave_front_costmap[map_pose_x][map_pose_y] = 0;
    //std::map<uint32_t, int> wave;
    //uint32_t index = mp_map->index(map_pose_x, map_pose_y);
    //int cost = 0;
    //wave[index] = cost;
    static int pos_array[4][2] = {{1, 0}, {0, 1},  {-1, 0}, {0, -1}};
    const uint32_t array_size = sizeof(pos_array) / sizeof(pos_array[0]);

    auto cmp = [&](const VectorX2<int>& a, const VectorX2<int>& b)
    {
        // 因为优先出列判定为!cmp，所以反向定义实现最小值优先
        return wave_front_costmap[a.x][a.y] > wave_front_costmap[b.x][b.y];
    };
    std::priority_queue<VectorX2<int>, std::vector<VectorX2<int>>, decltype(cmp)> qu(cmp);

    //std::queue<VectorX2<int>> qu;
    qu.push(VectorX2<int>(map_pose_x, map_pose_y));
    //uint32_t it = 0;
    uint32_t map_pose_start_x;
    uint32_t map_pose_start_y;
    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_start_x, &map_pose_start_y);
    bool is_find_path = false;
    while(!qu.empty())
    {

        VectorX2<int> map_pose = qu.top();
        qu.pop();
        //++cost;

        //printf(" ({}, {}) {:d}\n", map_pose.x, map_pose.y, m_wave_front_costmap[map_pose.x][map_pose.y]);
        if(map_pose.x == map_pose_start_x && map_pose.y == map_pose_start_y)
        {
            is_find_path = true;
            flow_field(wave_front_costmap, map_pose, p_path);
            return is_find_path;
        }
        //explore wave
        for(int i = 0; i < array_size; ++i)
        {
            //++it;
            VectorX2<int> next_map_pose;
            next_map_pose.x = map_pose.x + pos_array[i][0];
            next_map_pose.y = map_pose.y + pos_array[i][1];

            if(next_map_pose.x < map_w && next_map_pose.y < map_h)
            {
                //if(m_wave_front_costmap[next_map_pose.x][next_map_pose.y] == -1)
                {
                    int map_cost = (int)mp_map->cost((uint32_t)next_map_pose.x, (uint32_t)next_map_pose.y);

                    if(map_cost < obstacle_value)
                    {
                        int cost = wave_front_costmap[map_pose.x][map_pose.y] + 1 + map_cost * 0.1;
                        if(wave_front_costmap[next_map_pose.x][next_map_pose.y] == -1)
                        {
                            wave_front_costmap[next_map_pose.x][next_map_pose.y] = cost;
                            qu.push(next_map_pose);
                        }
                        else
                        {
                            if(cost < wave_front_costmap[next_map_pose.x][next_map_pose.y])
                            {
                                wave_front_costmap[next_map_pose.x][next_map_pose.y] = cost;
                                qu.push(next_map_pose);
                            }
                        }
                    }
                }
            }
        }
    }
    return is_find_path;
}

void WaveFrontPlanner::flow_field(const std::vector<std::vector<int> > &costmap,
                                  const VectorX2<int> &start_pose,
                                  std::vector<Pose<float> > *p_path)
{
    RECORD_TIME();
    VectorX2<int> cur_pose = start_pose;
    VectorX2<int> prev_pose = start_pose;
    VectorX2<int> next_pose = start_pose;
    do
    {
        int min_cost = std::numeric_limits<int>::max();
        int min_x = std::max(0, cur_pose.x - 1);
        int max_x = std::min((int)costmap.size(), cur_pose.x + 2);
        int min_y = std::max(0, cur_pose.y - 1);
        int max_y = std::min((int)costmap[0].size(), cur_pose.y + 2);

        for(int x = min_x; x < max_x; ++x)
        {
            for(int y = min_y; y < max_y; ++y)
            {
                if(x == cur_pose.x && y == cur_pose.y)
                {
                    continue;
                }
                if(x == prev_pose.x && y == prev_pose.y)
                {
                    continue;
                }
                if(costmap[x][y] != -1 &&
                    min_cost > costmap[x][y])
                {
                    min_cost = costmap[x][y];
                    next_pose.x = x;
                    next_pose.y = y;
                }
            }
        }
        prev_pose = cur_pose;
        cur_pose = next_pose;
        Pose<float> pose;
        mp_map->map_to_world((uint32_t)next_pose.x, (uint32_t)next_pose.y, &pose.position.x, &pose.position.y);
        p_path->emplace_back(pose);
    }
    while(costmap[cur_pose.x][cur_pose.y] > 0);
    smooth_path(mp_map, p_path);
    (*p_path)[0] = m_start_pose;
    p_path->back() = m_goal_pose;
}


void WaveFrontPlanner::inflate_path(const std::vector<Pose<float>> &original_path)
{
    RECORD_TIME();
    typedef std::array<uint32_t, 4> Cell;
    const uint32_t max_distance = 2 * m_akermann_wheel_base / tan(m_akermann_max_steer_angle) / mp_map->resolution();
    std::vector<Pose<float> > path = interpolate_path(original_path, mp_map->resolution());
    //const static int pos_array[8][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1},  {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
    const static int pos_array[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    const uint32_t array_size = sizeof(pos_array) / sizeof(pos_array[0]);
    std::vector<VectorX2<uint32_t>> map_path;
    std::queue<Cell> qu;
    uint32_t path_size = path.size();
    const uint8_t obstacle_value = mp_map->obstacles_cost()-1;
    uint32_t map_w = mp_map->size_in_cells_x();
    uint32_t map_h = mp_map->size_in_cells_y();
    const uint32_t angle_len = floor(2 * M_PI / m_angle_step);
    m_ref_path_costmap.resize(map_w);
    m_ref_path_3dcostmap.resize(map_w);
    for(uint32_t i = 0; i < map_w; ++i)
    {
        m_ref_path_costmap[i].assign(map_h, 255);
        m_ref_path_3dcostmap[i].resize(map_h);
        for(uint32_t j = 0; j < map_h; ++j)
        {
            m_ref_path_3dcostmap[i][j].assign(angle_len, 255);
        }
    }

    map_path.resize(path_size);
    for(uint32_t i = 0; i < path_size; ++i)
    {
        if(mp_map->world_to_map(path[i].position.x, path[i].position.y, &map_path[i].x, &map_path[i].y))
        {
            uint32_t angle_index = floor(constraint_angle_r(path[i].heading_angle) / m_angle_step);
            qu.push({0, map_path[i].x, map_path[i].y, angle_index});;
            m_ref_path_costmap[map_path[i].x][map_path[i].y] = 0;
            m_ref_path_3dcostmap[map_path[i].x][map_path[i].y][angle_index] = 0;
            continue;
        }
        else
        {
            PRINT_ERROR("path world pose to map error");
            return;
        }
    }

    while(!qu.empty())
    {
        Cell map_pose = qu.front();
        //VectorX3<uint32_t> map_pose = qu.front();
        qu.pop();

        for(int i = 0; i < array_size; ++i)
        {
            //++it;
            Cell next_map_pose;
            //distance
            next_map_pose[I_COST] = map_pose[I_COST] + 1;
            next_map_pose[I_X] = map_pose[I_X] + pos_array[i][0];
            next_map_pose[I_Y] = map_pose[I_Y] + pos_array[i][1];
            next_map_pose[I_HEADING] = map_pose[I_HEADING];
            if(next_map_pose[I_X] < map_w && next_map_pose[I_Y] < map_h)
            {
                if(m_ref_path_costmap[next_map_pose[I_X]][next_map_pose[I_Y]] == 255)
                {
                    uint8_t map_cost = mp_map->cost(next_map_pose[I_X], next_map_pose[I_Y]);
                    //const float drop_speed = 0.05;
                    const float drop_speed = 0.08;
                    int cost = obstacle_value-1 - exp(-1.0 * drop_speed * next_map_pose[I_COST]) * (obstacle_value-1);
                    if(cost < m_ref_path_costmap[next_map_pose[I_X]][next_map_pose[I_Y]] && map_cost < obstacle_value)
                    {
                        //添加最大膨胀距离，限制搜索范围
                        //if(next_map_pose[I_COST] < max_distance)
                        {
                            m_ref_path_costmap[next_map_pose[I_X]][next_map_pose[I_Y]] = cost;
                            qu.push(next_map_pose);
                            for(uint32_t j = angle_len; j < angle_len; ++j)
                            {
                                float delta_angle = (j - map_pose[I_HEADING]) * m_angle_step;
                                delta_angle = constraint_angle_r(delta_angle, -M_PI, M_PI);
                                uint32_t delta_angle_index = fabs(delta_angle) / m_angle_step;
                                m_ref_path_3dcostmap[next_map_pose[I_X]][next_map_pose[I_Y]][j] = cost + delta_angle_index * 5;
                            }
                        }

                    }
                }
            }
        }
    }

#define DEBUG_WRITE_COST 1
#ifdef DEBUG_WRITE_COST
    uint32_t size = map_w * map_h;
    uint8_t *map_data = (uint8_t*)malloc(size);
    for(int i = 0; i < size; ++i)
    {
        uint32_t y = i / map_w;
        uint32_t x = i - y * map_w;
        y = map_h - y;
        int value = m_ref_path_costmap[x][y];
        if (value == 255)
        {
            map_data[i] = 0;
        }
        else
        {
            map_data[i] = 255 - value * 255.0 / obstacle_value;
        }
        //map_data[i] = 255 - value;
    }
    write_pnm_image(map_data, map_w, map_h, "/home/zhou/local_map.pgm", pnm_t::P5);
    free(map_data);
#endif
}

bool WaveFrontPlanner::sorted_wave_front3(Pose<float> start_pose, Pose<float> goal_pose, std::vector<Pose<float> > *p_path)
{
#if 1
    RECORD_TIME();
    // cost x y heading
    //typedef std::array<uint32_t, 4> Cell;
    class Cell{
    public:
        Cell(const uint32_t &ix, const uint32_t &iy, const uint32_t &iheading, const float &icost):
            x(ix),
            y(iy),
            heading_index(iheading),
            cost(icost)
                {}
        Cell() {}
    public:
        uint32_t x;
        uint32_t y;
        uint32_t heading_index;
        float cost;
    };
    uint32_t map_pose_x;
    uint32_t map_pose_y;
    uint32_t it = 0;
    uint32_t map_pose_goal_x;
    uint32_t map_pose_goal_y;
    uint32_t re_sort_counter = 0;
    uint32_t skip_it = 0;
    bool is_find_path = false;
    const uint32_t look_up_table_size = m_look_up_table[0].size();
    const uint32_t angle_index_range = m_look_up_table.size();
    //std::swap(start_pose, goal_pose);

    //const int8_t obstacle_value = mp_map->obstacles_cost()-1;
    m_wave_map.clear();
    m_wave_front_prev_node.clear();
    m_map_w = mp_map->size_in_cells_x();
    m_map_h = mp_map->size_in_cells_y();
    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_x, &map_pose_y);
    start_pose.heading_angle = constraint_angle_r(start_pose.heading_angle);
    uint32_t header_index = (uint32_t)round(start_pose.heading_angle / m_angle_step) % angle_index_range;
    uint64_t index = pose_to_index(map_pose_x, map_pose_y, header_index);
    m_wave_map[index][I_COST] = 0;
    m_wave_map[index][I_X] = map_pose_x;
    m_wave_map[index][I_Y] = map_pose_y;
    m_wave_map[index][I_HEADING] = start_pose.heading_angle;

    auto cmp = [this](const Cell& a, const Cell& b)
    {
        const float ratio = m_distance_cost_ratio;
        // fmm
        return a.cost + m_fmm_look_up_table[a.x][a.y] * ratio
               > b.cost + m_fmm_look_up_table[b.x][b.y] * ratio;
    };

    //std::priority_queue<Pose<float>, std::vector<Pose<float>>, decltype(cmp)> qu(cmp);
    std::priority_queue<Cell, std::vector<Cell>, decltype(cmp)> qu(cmp);
    //boost::heap::binomial_heap<Pose<float>, boost::heap::compare<decltype(cmp)>> qu(cmp);
    //boost::heap::priority_queue<Pose<float>, boost::heap::compare<decltype(cmp)>> qu(cmp);
    //std::queue<VectorX2<int>> qu;
    PRINT_INFO("start node: {:d}, {:d}, {:d}, {}", map_pose_x, map_pose_y, header_index, index);
    qu.push(Cell(map_pose_x, map_pose_y, header_index, 0));
    goal_pose.heading_angle = constraint_angle_r(goal_pose.heading_angle) ;
    uint32_t map_pose_goal_heading_index = (uint32_t)round(goal_pose.heading_angle / m_angle_step) % angle_index_range;
    mp_map->world_to_map(goal_pose.position.x, goal_pose.position.y, &map_pose_goal_x, &map_pose_goal_y);
    uint64_t goal_index = pose_to_index(Pose<int>(map_pose_goal_x, map_pose_goal_y, map_pose_goal_heading_index));
    PRINT_DEBUG("goal({}, {}, {}) : {}", map_pose_goal_x, map_pose_goal_y, map_pose_goal_heading_index, goal_index);

    uint32_t max_qu_size = qu.size();
    p_path->clear();
    if(m_flag_stop)
    {
        m_flag_stop = false;
        PRINT_DEBUG("planner stop");
        return false;
    }
    while(!qu.empty())
    {
        //PRINT_DEBUG("searching ..., it = {}, flag_stop = {}", it, m_flag_stop);
        if(it > 100)
        {
            it = 0;
            if(m_flag_stop)
            {
                m_flag_stop = false;
                PRINT_DEBUG("planner stop");
                return false;
            }
        }
        Cell cur_cell = qu.top();
        qu.pop();
        //PRINT_DEBUG("cur_cell:({}, {}, {}, {})", cur_cell.x, cur_cell.y, cur_cell.heading_index, cur_cell.cost);
        //Pose<int> map_pose_index(round(map_pose.position.x), round(map_pose.position.y), round(map_pose.heading_angle/m_angle_step));
        //Pose<int> map_pose_index(int(map_pose.position.x), int(map_pose.position.y), round(map_pose.heading_angle/m_angle_step));
        uint64_t cur_index = pose_to_index(cur_cell.x, cur_cell.y, cur_cell.heading_index);
        if(cur_index == goal_index)
        {
            PRINT_INFO("find akermann path\n\n\n");
            //            PRINT_DEBUG("(%f, %f, %f) %f", map_pose.position.x, map_pose.position.y, map_pose.heading_angle,
            //                        m_wave_front_costmap[map_pose.position.x][map_pose.position.y][map_pose.heading_angle][0]);
            is_find_path = true;
            generate_path(cur_index, p_path);
            PRINT_INFO("re_sort_counter = {}, skip {:d}, max_qu_size = {}, it = {}", re_sort_counter, skip_it, max_qu_size, it);
            return is_find_path;
        }

        //explore wave
        //if(cur_cell.cost > m_wave_map[cur_index][I_COST])
        if(m_wave_map[cur_index][I_IS_CLOSED] != 0)
        {
            ++skip_it;
            continue;
        }
        m_wave_map[cur_index][I_IS_CLOSED] = 1;
        for(int i = 0; i < look_up_table_size; ++i)
        {
            ++it;
            //PRINT_DEBUG("m_look_up_table.size = {}, cur_cell.heading_index = {}", m_look_up_table.size(), cur_cell.heading_index);
            float next_x = m_wave_map[cur_index][I_X] + m_look_up_table[cur_cell.heading_index][i].delta_x;
            float next_y = m_wave_map[cur_index][I_Y] + m_look_up_table[cur_cell.heading_index][i].delta_y;
            float next_heading = constraint_angle_r(m_wave_map[cur_index][I_HEADING] + m_look_up_table[cur_cell.heading_index][i].delta_heading);
            //float steer = m_look_up_table[map_pose.heading_angle][i].steer_angle;
            //Pose<float> next_map_pose(next_x, next_y, next_heading);
            //Pose<int> next_map_pose_index;
            //            next_map_pose_index.position.x = round(next_x);
            //            next_map_pose_index.position.y = round(next_y);
            //            next_map_pose_index.heading_angle = round(next_heading / m_angle_step);
            Cell next_cell;
            next_cell.x = uint32_t(next_x);
            next_cell.y = uint32_t(next_y);
            next_cell.heading_index = (uint32_t)round(next_heading / m_angle_step) % angle_index_range;

            //PRINT_DEBUG("next_heading = {}, next_cell.heading_index = {}", next_heading, next_cell.heading_index);
            if(next_cell.x < m_map_w && next_cell.y < m_map_h)
            {
                //if(m_wave_front_costmap[next_map_pose.x][next_map_pose.y] == -1)
                {
                    uint8_t map_cost = mp_map->cost(next_cell.x, next_cell.y);
                    //wavefront
                    //if(m_wave_front_look_up_table[next_cell[I_X]][next_cell.y] != -1)
                    //fmm
                    if(m_fmm_look_up_table[next_cell.x][next_cell.y] != INF)
                    {
                        //float steer_angle = fabs(m_look_up_table[map_pose.heading_angle][i].steer_angle);
                        uint64_t next_index = pose_to_index(next_cell.x, next_cell.y, next_cell.heading_index);

                        float cost = m_wave_map[cur_index][I_COST] + (float)m_step_cost_ratio * m_look_up_table[cur_cell.heading_index][i].length;
                        //cost += int(fabs(steer_angle) * steer_angle_ratio);
                        cost += (float)m_delta_heading_cost_ratio * fabs(m_look_up_table[cur_cell.heading_index][i].delta_heading);
                        cost += (float)(map_cost * m_map_cost_ratio);
                        //cost = m_wave_map[cur_index][0] + 10;

                        if(m_wave_map.find(next_index) == m_wave_map.end())
                        {
                            m_wave_map[next_index][I_COST] = cost;
                            m_wave_map[next_index][I_X] = next_x;
                            m_wave_map[next_index][I_Y] = next_y;
                            m_wave_map[next_index][I_HEADING] = next_heading;
                            //PRINT_DEBUG("(%f %f, %f) %lu", next_x, next_y, next_heading * 180 * M_1_PI, next_index);
                            next_cell.cost = cost;
                            qu.push(next_cell);
                            m_wave_front_prev_node[next_index] = cur_index;
                        }
                        else
                        {
                            if(cost < m_wave_map[next_index][0])
                            {
                                //PRINT_DEBUG("update cost\n\n\n\n");
                                m_wave_map[next_index][I_COST] = cost;
                                m_wave_map[next_index][I_X] = next_x;
                                m_wave_map[next_index][I_Y] = next_y;
                                m_wave_map[next_index][I_HEADING] = next_heading;
                                //PRINT_DEBUG("(%f %f, %f) %lu", next_x, next_y, next_heading * 180 * M_1_PI, next_index);
                                next_cell.cost = cost;
                                qu.push(next_cell);
                                m_wave_front_prev_node[next_index] = cur_index;
                                ++re_sort_counter;
                            }
                        }

//                        //for test
//                        if(qu.size() > max_qu_size)
//                        {
//                            max_qu_size = qu.size();
//                        }
                    }
                }
            }
        }
    }
    //PRINT_INFO("re_sort_counter = {}, max_qu_size = {}, it = {}", re_sort_counter, max_qu_size, it);
    PRINT_ERROR("can't find path !!!\n\n\n");
    return is_find_path;
#endif
    return false;
}

void WaveFrontPlanner::stop()
{
    PRINT_DEBUG("stop");
    m_flag_stop = true;
//    while (m_flag_stop == false)
//    {
//        break;
//    }
//    PRINT_DEBUG("stop finished");
//    return;
}

} // namespace hybrid_astar_planner
}
