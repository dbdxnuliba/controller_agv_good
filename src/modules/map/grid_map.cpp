#include "grid_map.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <queue>
#include <cassert>
#include "common/common.h"
#include "common/print.h"
#include "common/pnm_image.h"
#include "common/time_keeper.h"

namespace bz_robot
{
GridMap::GridMap()
{
    m_is_static = false;
    m_is_enable_obstacle_map = false;
    m_is_enable_inflaction = false;
    m_obstacles_map_over_time_ms = 3 * 1000;
}

GridMap::~GridMap()
{

}

void GridMap::set_incoming_map_data(const int8_t *p_incoming_map_data, const float resolution, const uint32_t x_size, const uint32_t y_size, const float origin_x, const float origin_y)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_incoming_map.resolution = resolution;
    m_incoming_map.x_size = x_size;
    m_incoming_map.y_size = y_size;
    m_incoming_map.origin_x = origin_x;
    m_incoming_map.origin_y = origin_y;
    m_incoming_map.map.resize(x_size);

    for(uint32_t i = 0; i < x_size; i++)
    {
        m_incoming_map.map[i].resize(y_size);
    }
    for(uint32_t i = 0; i < x_size * y_size; i++)
    {
        uint32_t y = i / x_size;
        uint32_t x = i - y * x_size;
        m_incoming_map.map[x][y] = p_incoming_map_data[i];
        printf("%d ", p_incoming_map_data[i]);
        if(p_incoming_map_data[i] == 100)
        {
            m_incoming_map.obstacles_index_set.insert((uint64_t)i);
        }
    }
}

void GridMap::set_incoming_map_data(const GridMapData &incoming_map)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_incoming_map = incoming_map;
    for(uint32_t x = 0; x < m_incoming_map.x_size; ++x)
    {
        for(uint32_t y = 0; y < m_incoming_map.x_size; ++y)
        {
            if(m_incoming_map.map[x][y] == 100)
            {
                uint64_t index = y * m_incoming_map.x_size + x;
                m_incoming_map.obstacles_index_set.insert((uint64_t)index);
            }
        }
    }
}

void GridMap::set_resolution(const float resolution)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_map.resolution = resolution;
    m_map.origin_x = m_incoming_map.origin_x;
    m_map.origin_y = m_incoming_map.origin_y;
    PRINT_DEBUG("m_incoming_map.x_size = {:d}, m_incoming_map.y_size = {:d}", m_incoming_map.x_size, m_incoming_map.y_size);
    PRINT_DEBUG("m_incoming_map.resolution = {:f}, resolution = {:f}", m_incoming_map.resolution, resolution);
    PRINT_DEBUG("m_incoming_map.x_size * m_incoming_map.resolution / resolution = {:f}", m_incoming_map.x_size * m_incoming_map.resolution / resolution);
    PRINT_DEBUG("m_incoming_map.y_size * m_incoming_map.resolution / resolution = {:f}", m_incoming_map.y_size * m_incoming_map.resolution / resolution);
    m_map.x_size = round(ceil(m_incoming_map.x_size * m_incoming_map.resolution / resolution * 100) / 100.0);
    m_map.y_size = round(ceil(m_incoming_map.y_size * m_incoming_map.resolution / resolution * 100) / 100.0);;
    /*
     * Reasons for ceil(x*100)/100;
        [DEBUG]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map.cpp]-[set_resolution]-[49]: m_incoming_map.x_size = 868, m_incoming_map.y_size = 629
        [DEBUG]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map.cpp]-[set_resolution]-[50]: m_incoming_map.resolution = 0.100000, resolution = 0.100000
        [DEBUG]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map.cpp]-[set_resolution]-[51]: m_incoming_map.x_size * m_incoming_map.resolution / resolution = 868.000000
        [DEBUG]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map.cpp]-[set_resolution]-[52]: m_incoming_map.y_size * m_incoming_map.resolution / resolution = 629.000000
        [DEBUG]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map.cpp]-[set_resolution]-[55]: m_x_size = 868, m_y_size = 629
        [INFO]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map_server.cpp]-[on_receive_incoming_map]-[69]: receive local map

        [DEBUG]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map.cpp]-[set_resolution]-[49]: m_incoming_map.x_size = 868, m_incoming_map.y_size = 629
        [DEBUG]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map.cpp]-[set_resolution]-[50]: m_incoming_map.resolution = 0.100000, resolution = 0.100000
        [DEBUG]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map.cpp]-[set_resolution]-[51]: m_incoming_map.x_size * m_incoming_map.resolution / resolution = 868.000013
        [DEBUG]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map.cpp]-[set_resolution]-[52]: m_incoming_map.y_size * m_incoming_map.resolution / resolution = 629.000009
        [DEBUG]-[/home/zhou/catkin_ws/src/hybrid_astar/src/my_map/grid_map.cpp]-[set_resolution]-[55]: m_x_size = 869, m_y_size = 630

     */

    PRINT_DEBUG("m_x_size = {:d}, m_y_size = {:d}", m_map.x_size, m_map.y_size);
    m_map.map.resize(m_map.x_size);
    m_empty_map.resize(m_map.x_size);
    for(uint32_t i = 0; i < m_map.x_size; i++)
    {
        m_map.map[i].resize(m_map.y_size);
        m_empty_map[i].assign(m_map.y_size, 0);
    }
    m_obstacle_map = m_empty_map;
    for(uint32_t x = 0; x < m_map.x_size; x++)
    {
        for (uint32_t y = 0; y < m_map.y_size; y++)
        {
            uint32_t start_x = floor(x * resolution / m_incoming_map.resolution);
            uint32_t end_x = ceil((x+1) * resolution / m_incoming_map.resolution);
            end_x = std::min(end_x, m_incoming_map.x_size-1);
            uint32_t start_y = floor(y * resolution / m_incoming_map.resolution);
            uint32_t end_y = ceil((y+1) * resolution / m_incoming_map.resolution);
            end_y = std::min(end_y, m_incoming_map.y_size-1);
            int8_t max_value = -1;
            for(uint32_t i = start_x; i < end_x; i++)
            {
                for(uint32_t j = start_y; j < end_y; j++)
                {
                    max_value = std::max(max_value, m_incoming_map.map[i][j]);
                }
            }
            int8_t cost = max_value;

            m_map.map[x][y] = cost;
        }
    }
    m_original_map = m_map;

}

float GridMap::resolution()
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    return m_map.resolution;
}

void GridMap::update_map()
{
    if(m_is_enable_inflaction)
    {
        if(m_is_enable_obstacle_map)
        {
            update_obstacles_map();
            inflation_obstacles(&m_obstacle_map, m_original_map.x_size, m_original_map.y_size);
        }
    }
    if(m_is_enable_obstacle_map)
    {
        for(uint32_t x = 0; x < m_map.x_size; x++)
        {
            for(uint32_t y = 0; y < m_map.y_size; y++)
            {
                m_map.map[x][y] = std::max(m_original_map.map[x][y], m_obstacle_map[x][y]);
            }
        }
    }
    else
    {
        m_map.map = m_original_map.map;
    }

}

void GridMap::map_data(GridMapData *p_grid_map_data)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    *p_grid_map_data = m_map;
}

void GridMap::set_robot_contour(const std::vector<VectorX2<float> > contour_list)
{
    m_map.robot_contour = contour_list;
    m_robot_contour.set_contour(contour_list);
}

std::vector<VectorX2<float> > GridMap::robot_contour()
{
    return m_robot_contour.contour();
}

void GridMap::generate_empty_map(std::vector<std::vector<int8_t>> *p_map)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    *p_map = m_empty_map;
}

bool GridMap::set_cost(const uint32_t mx, const uint32_t my, int8_t value)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    if(mx < m_map.x_size && my < m_map.y_size)
    {
        m_map.map[mx][my] = value;
        return true;
    }
    return false;
}

int8_t GridMap::cost(const uint32_t mx, const uint32_t my)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    assert(mx < m_map.x_size);
    assert(my < m_map.y_size);
    return m_map.map[mx][my];
}

uint32_t GridMap::index(const uint32_t mx, const uint32_t my)
{
    return m_map.x_size * my + mx;
}

void GridMap::index_to_cells(uint32_t index, uint32_t *mx, uint32_t *my)
{
    uint32_t x_size = 0;
    uint32_t y_size = 0;
    cell_size(&x_size, &y_size);

    *my = index / x_size;
    *mx = index - *my * x_size;
}


void GridMap::original_point(float *p_wx, float *p_wy)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    assert(p_wx != nullptr);
    assert(p_wx != nullptr);
    *p_wx = m_map.origin_x;
    *p_wy = m_map.origin_y;
}

void GridMap::cell_size(uint32_t *p_mx, uint32_t *p_my)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    assert(p_mx != nullptr);
    assert(p_my != nullptr);
    *p_mx = m_map.x_size;
    *p_my = m_map.y_size;
}

void GridMap::world_size(float *p_wx, float *p_wy)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    assert(p_wx != nullptr);
    assert(p_wy != nullptr);
    *p_wx = m_incoming_map.x_size * m_incoming_map.resolution;
    *p_wy = m_incoming_map.y_size * m_incoming_map.resolution;
}

void GridMap::set_robot_pose(const float x, const float y, const float heading_angle)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_robot_x = x;
    m_robot_y = y;
    m_robot_heading_angle = heading_angle;
}

bool GridMap::map_to_world(const uint32_t mx, const uint32_t my, float *p_wx, float *p_wy)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    return map_to_world(mx, my, p_wx, p_wy);
//    if (mx < m_map.x_size && my < m_map.y_size)
//    {
//        *p_wx = m_incoming_map.origin_x + (mx) * m_map.resolution;
//        *p_wy = m_incoming_map.origin_y + (my) * m_map.resolution;
//        return true;
//    }
//    return false;
}

bool GridMap::world_to_map(const float wx, const float wy, uint32_t *p_mx, uint32_t *p_my)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    return __world_to_map(wx, wy, p_mx, p_my);
//    if (wx < m_incoming_map.origin_x || wy < m_incoming_map.origin_y)
//        return false;

//    *p_mx = (uint32_t)((wx - m_incoming_map.origin_x) / m_map.resolution);
//    *p_my = (uint32_t)((wy - m_incoming_map.origin_y) / m_map.resolution);

//    if (*p_mx < m_map.x_size && *p_my < m_map.y_size)
//    {
//        return true;
//    }
//    return false;
}


void GridMap::set_inflation_radius(const float inflaction_radius, const float drop_speed)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);

    m_cell_inflation_radius = ceil(inflaction_radius / m_map.resolution);
//    PRINT_DEBUG("m_cell_inflation_radius = {:d}", m_cell_inflation_radius);
    m_cached_costs.resize(m_cell_inflation_radius + 2);
    m_cached_distances.resize(m_cell_inflation_radius + 2);
    for(uint32_t i = 0; i <= m_cell_inflation_radius + 1; ++i)
    {
        m_cached_costs[i].resize(m_cell_inflation_radius + 2, 0);
        m_cached_distances[i].assign(m_cell_inflation_radius + 2, 0);
        for(uint32_t j = 0; j <= m_cell_inflation_radius + 1; ++j)
        {
            uint32_t distance = floor(hypot(i, j));
            m_cached_distances[i][j] = distance;

            if(distance == 0)
            {
                m_cached_costs[i][j] = obstacles_cost();
            }
            else if(hypot(i, j) * resolution() <= m_robot_contour.inscribed_radius())
            {
                m_cached_costs[i][j] = obstacles_cost() - 1;
            }
            else
            {
                const float factor =  exp(-1.0 * drop_speed * (hypot(i, j) * resolution() - m_robot_contour.inscribed_radius()));
                m_cached_costs[i][j] = (int8_t)(obstacles_cost() - 2) * factor;
            }
        }
    }
}

inline void GridMap::set_static(bool is_static)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_is_static = is_static;
}

inline bool GridMap::is_static()
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    return m_is_static;
}

inline void GridMap::enable_inflaction(bool is_inflaction_enabled)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_is_enable_inflaction = is_inflaction_enabled;
    if(m_is_enable_inflaction)
    {
        inflation_obstacles(&m_original_map.map, m_original_map.x_size, m_original_map.y_size);
    }
}

inline bool GridMap::is_inflaction_enabled()
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    return m_is_enable_inflaction;
}

void GridMap::insert_obstacle_map(const std::vector<std::vector<int8_t> > &map)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_obstacle_map = map;
}

inline void GridMap::enable_obstacle_map(bool is_obstacle_map_enabled)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_is_enable_obstacle_map = is_obstacle_map_enabled;
}

inline bool GridMap::is_obstacle_map_enabled()
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    return m_is_enable_obstacle_map;
}

inline void GridMap::set_obstacles_cost(const int8_t value)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_obstacles_cost = value;
}

inline int8_t GridMap::obstacles_cost()
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    return m_obstacles_cost;
}

inline bool GridMap::is_obstacles(const uint32_t mx, const uint32_t my)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    return m_map.map[mx][my] == obstacles_cost();
}

void GridMap::set_obstacles_map_over_time(const uint32_t &over_time_ms)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_obstacles_map_over_time_ms = over_time_ms;
}

inline bool GridMap::__map_to_world(const uint32_t mx, const uint32_t my, float *p_wx, float *p_wy)
{
    if (mx < m_map.x_size && my < m_map.y_size)
    {
        *p_wx = m_incoming_map.origin_x + (mx) * m_map.resolution;
        *p_wy = m_incoming_map.origin_y + (my) * m_map.resolution;
        return true;
    }
    return false;
}

inline bool GridMap::__world_to_map(const float wx, const float wy, uint32_t *p_mx, uint32_t *p_my)
{
    *p_mx = (uint32_t)((wx - m_incoming_map.origin_x) / m_map.resolution);
    *p_my = (uint32_t)((wy - m_incoming_map.origin_y) / m_map.resolution);

    if (*p_mx < m_map.x_size && *p_my < m_map.y_size)
    {
        return true;
    }
    return false;
}

inline void GridMap::insert_obstacles(const Obstacle& obstacle)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    for(auto it = m_obstacles_map_queue.begin(); it != m_obstacles_map_queue.end(); ++it)
    {
        if(it->time_stamp_us == obstacle.time_stamp_us && it->name == obstacle.name)
        {
            return;
        }
    }
    m_obstacles_map_queue.emplace_back(obstacle);
    return;
}

void GridMap::update_obstacles_map()
{
    //RECORD_TIME();
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    const int64_t time_stamp = time_stamp_us();
    //PRINT_DEBUG("cur time stamp = {:.3f}",  1e-6 * time_stamp);
    //if m_obstacles_map_over_time_ms == 0. use last obstacles data
    if(m_obstacles_map_over_time_ms == 0)
    {
        while (m_obstacles_map_queue.size() > 1)
        {
            m_obstacles_map_queue.pop_front();
        }
    }
    else
    {
        while (0 < m_obstacles_map_queue.size())
        {
            //PRINT_DEBUG("check: {:.3f}", 1e-6 * m_obstacles_map_queue[0].first);
            if(time_stamp - m_obstacles_map_queue[0].time_stamp_us > (int64_t)m_obstacles_map_over_time_ms * 1000)
            {
                m_obstacles_map_queue.pop_front();
            }
            else
            {
                break;
            }
        }
        //PRINT_DEBUG("check finished");
    }

    m_obstacle_map = m_empty_map;
    const int8_t obstacles_value = obstacles_cost();
    uint32_t map_x = 0;
    uint32_t map_y = 0;
    //PRINT_DEBUG("m_obstacles_map_queue size = {}", m_obstacles_map_queue.size());
    for(int i = 0; i < m_obstacles_map_queue.size(); ++i)
    {
        const std::vector<VectorX2<FLOAT_T>> &obstacles = m_obstacles_map_queue[i].data;
        for(int j = 0; j < obstacles.size(); ++j)
        {
            if(__world_to_map(obstacles[j].x, obstacles[j].y, &map_x, &map_y))
            {
                m_obstacle_map[map_x][map_y] = obstacles_value;
            }
        }
    }
}

#if 1
void GridMap::inflation_obstacles(std::vector<std::vector<int8_t>> *p_map, const uint32_t map_x_size, const uint32_t map_y_size)
{
    //RECORD_TIME();
    uint32_t it = 0;
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    //typedef std::array<uint32_t, 3> Cell;
    const static int pos_array[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    //const static int pos_array[8][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1},  {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
    const uint32_t array_size = sizeof(pos_array) / sizeof(pos_array[0]);
    std::queue<CellData2> qu;
    std::vector<std::vector<bool>> is_explore_map;
    const int obstacles_value = obstacles_cost();

    is_explore_map.resize(map_x_size);
    for(uint32_t x = 0; x < map_x_size; ++x)
    {
        is_explore_map[x].assign(map_y_size, false);
        for(uint32_t y = 0; y < map_y_size; ++y)
        {
            int8_t cost = (*p_map)[x][y];
            if(cost == obstacles_value)
            {
                qu.push(CellData2(x, y, 0, 0));
                (*p_map)[x][y] = m_cached_costs[0][0];
            }
        }
    }

    while(!qu.empty())
    {
        CellData2 map_pose = qu.front();
        //VectorX3<uint32_t> map_pose = qu.front();
        qu.pop();
        uint32_t x = map_pose.x + map_pose.dx;
        uint32_t y = map_pose.y + map_pose.dy;

        if(is_explore_map[x][y])
        {
            continue;
        }
        is_explore_map[x][y] = true;
        for(int i = 0; i < array_size; ++i)
        {
            ++it;
            CellData2 next_map_pose;
            next_map_pose.x = map_pose.x;
            next_map_pose.y = map_pose.y;
            next_map_pose.dx = map_pose.dx + pos_array[i][0];
            next_map_pose.dy = map_pose.dy + pos_array[i][1];
            int dx = abs(next_map_pose.dx);
            int dy = abs(next_map_pose.dy);

            uint32_t distance = m_cached_distances[dx][dy];
            uint32_t next_x = map_pose.x + next_map_pose.dx;
            uint32_t next_y = map_pose.y + next_map_pose.dy;
            if(next_x < map_x_size && next_y < map_y_size)
            {
                if(distance <= m_cell_inflation_radius && (*p_map)[next_x][next_y] < 1)
                {
                    qu.push(next_map_pose);
                    (*p_map)[next_x][next_y] = m_cached_costs[dx][dy];
                }
            }
        }
    }
    return;
}

#else
void GridMap::inflation_obstacles(std::vector<std::vector<int8_t>> *p_map, const uint32_t map_x_size, const uint32_t map_y_size)
{
    //RECORD_TIME();
    uint32_t it = 0;
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);

    std::map<uint32_t, std::vector<CellData> > inflation_cells;
    //init map
    for(uint32_t i = 0; i < m_cached_distances.size(); i++)
    {
        for(uint32_t j = 0; j < m_cached_distances[i].size(); j++)
        {
            std::vector<CellData> cell_data_vector;
            cell_data_vector.clear();
            inflation_cells[m_cached_distances[i][j]] = cell_data_vector;
        }
    }
    inflation_cells[0] = std::vector<CellData>();

    uint32_t seen_size = map_x_size * map_y_size;

    m_seen.assign(seen_size, false);
    std::vector<CellData> &obs_bin = inflation_cells[0];

//    uint32_t min_j = std::min(map_x_size, map_y_size);
    for(uint32_t j = 0; j < map_y_size; j++)
    {
        for(uint32_t i = 0; i < map_x_size; i++)
        {
            int index = j * map_x_size + i;
            int8_t cost = (*p_map)[i][j];
            if(cost == obstacles_cost())
            {
                obs_bin.push_back(CellData(index, i, j, i, j));
            }
        }
    }

    //std::map<uint32_t, std::vector<CellData> >::iterator bin;
    for(auto bin = inflation_cells.begin(); bin != inflation_cells.end(); bin++)
    {
        for(int i = 0; i < bin->second.size(); ++i)
        {
            // process all cells at distance dist_bin.first
            const CellData &cell = bin->second[i];

            uint32_t index = cell.index;
            // ignore if already visited
            if(m_seen[index])
            {
                continue;
            }
            m_seen[index] = true;

            uint32_t mx = cell.x;
            uint32_t my = cell.y;
            uint32_t sx = cell.src_x;
            uint32_t sy = cell.src_y;

            // assign the cost associated with the distance from an obstacle to the cell
            uint32_t dx = abs((int)((int)mx - (int)sx));
            uint32_t dy = abs((int)((int)my - (int)sy));
            int8_t cost = m_cached_costs[dx][dy];
            uint32_t y = index / map_x_size;
            uint32_t x = index - y * map_x_size;

            int8_t old_cost = (*p_map)[x][y];

            if(old_cost != -1)
            {
                (*p_map)[x][y] = std::max(old_cost, cost);
            }

            // attempt to put the neighbors of the current cell onto the inflation list
            if(mx > 0)
            {
                //x-1, y
                enqueue(index - 1, mx - 1, my, sx, sy, &inflation_cells);
            }
            if(my > 0)
            {
                //x, y-1
                enqueue(index - map_x_size, mx, my - 1, sx, sy, &inflation_cells);
            }

            if(mx < map_x_size - 1)
            {
                //x+1, y
                enqueue(index + 1, mx + 1, my, sx, sy, &inflation_cells);
            }

            if(my < map_y_size - 1)
            {
                //x, y+1
                enqueue(index + map_x_size, mx, my + 1, sx, sy, &inflation_cells);
            }
            it +=4;
        }
    }
    //PRINT_INFO("it = {}", it);
}
#endif
inline void GridMap::enqueue(uint32_t index, uint32_t mx, uint32_t my, uint32_t src_x, uint32_t src_y, std::map<uint32_t, std::vector<CellData>> *p_inflation_cells)
{
    if(!m_seen[index])
    {
        uint32_t dx = abs((int)((int)mx - (int)src_x));
        uint32_t dy = abs((int)((int)my - (int)src_y));
        uint32_t distance = m_cached_distances[dx][dy];
        // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
        if(distance > m_cell_inflation_radius)
        {
            return;
        }
        (*p_inflation_cells)[m_cached_distances[dx][dy]].push_back(CellData(index, mx, my, src_x, src_y));
    }
}

}
