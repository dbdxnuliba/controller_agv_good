#include "local_grid_map.h"
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <cassert>
#include "common/print.h"

namespace bz_robot
{
LocalGridMap::LocalGridMap() : GridMap ()
{
    m_rolling_window_x_size = 0;
    m_rolling_window_y_size = 0;
}

LocalGridMap::~LocalGridMap()
{

}

void LocalGridMap::update_map()
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    generate_rolling_window_map(m_rolling_window_x_size, m_rolling_window_y_size);
    //merge obstacles map;
    if(m_is_enable_obstacle_map)
    {
        update_obstacles_map();
        for(uint32_t x = 0; x < m_rolling_map.x_size; x++)
        {
            for(uint32_t y = 0; y < m_rolling_map.y_size; y++)
            {
                m_rolling_map.map[x][y] = std::max(m_rolling_map.map[x][y], m_obstacle_map[x][y]);
            }
        }
    }
    if(m_is_enable_inflaction)
    {
        inflation_obstacles(&m_rolling_map.map, m_rolling_map.x_size, m_rolling_map.y_size);
    }
}

void LocalGridMap::map_data(GridMapData *p_grid_map_data)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    *p_grid_map_data = m_rolling_map;
}

void LocalGridMap::set_robot_contour(const std::vector<VectorX2<float> > contour_list)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_rolling_map.robot_contour = contour_list;
    m_robot_contour.set_contour(contour_list);
}

void LocalGridMap::generate_rolling_window_map(float window_x_size, float window_y_size)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);

    const float world_x_size = m_incoming_map.x_size * m_incoming_map.resolution;
    const float world_y_size = m_incoming_map.y_size * m_incoming_map.resolution;

    float min_w_x = m_robot_x - window_x_size * 0.5;
    float max_w_x = m_robot_x + window_x_size * 0.5;
    float min_w_y = m_robot_y - window_y_size * 0.5;
    float max_w_y = m_robot_y + window_y_size * 0.5;

    if(min_w_x < m_incoming_map.origin_x)
    {
        min_w_x = m_incoming_map.origin_x;
        max_w_x = min_w_x + window_x_size;
    }
    if(min_w_y < m_incoming_map.origin_y)
    {
        min_w_y = m_incoming_map.origin_y;
        max_w_y = min_w_y + window_y_size;
    }
    if(max_w_x > m_incoming_map.origin_x + world_x_size)
    {
        max_w_x = m_incoming_map.origin_x + world_x_size;
        min_w_x = max_w_x - window_x_size;
    }
    if(max_w_y > m_incoming_map.origin_y + world_y_size)
    {
        max_w_y = m_incoming_map.origin_y + world_y_size;
        min_w_y = max_w_y - window_y_size;
    }

    uint32_t min_m_x = 0;
    uint32_t max_m_x = 0;
    uint32_t min_m_y = 0;
    uint32_t max_m_y = 0;
    min_m_x = (uint32_t)((min_w_x - m_incoming_map.origin_x) / m_map.resolution);
    min_m_y = (uint32_t)((min_w_y - m_incoming_map.origin_y) / m_map.resolution);
    max_m_x = min_m_x + m_rolling_map.x_size;
    max_m_y = min_m_y + m_rolling_map.y_size;

    m_rolling_map.origin_x = m_incoming_map.origin_x + (min_m_x) * m_map.resolution;
    m_rolling_map.origin_y = m_incoming_map.origin_y + (min_m_y) * m_map.resolution;

    m_rolling_map.map.resize(m_rolling_map.x_size);
    for(uint32_t i = 0; i < m_rolling_map.x_size; i++)
    {
        m_rolling_map.map[i].resize(m_rolling_map.y_size);
    }
    for(uint32_t x = 0; x < m_rolling_map.x_size; x++)
    {
        for(uint32_t y = 0; y < m_rolling_map.y_size; y++)
        {
            m_rolling_map.map[x][y] = m_map.map[x+min_m_x][y+min_m_y];
        }
    }

//    PRINT_DEBUG("m_robot_x = %f, m_robot_y = %f", m_robot_x, m_robot_y);
//    PRINT_DEBUG("m_rolling_map.origin_x = %f, m_rolling_map.origin_y = %f",
//                m_rolling_map.origin_x, m_rolling_map.origin_y);
    uint32_t map_robot_x = 0;
    uint32_t map_robot_y = 0;
    GridMap::world_to_map(m_robot_x, m_robot_y, &map_robot_x, &map_robot_y);
    float world_robot_x = 0;
    float world_robot_y = 0;
    GridMap::map_to_world(map_robot_x, map_robot_y, &world_robot_x, &world_robot_y);
//    PRINT_DEBUG("world_robot_x = %f, world_robot_y = %f", world_robot_x, world_robot_y);
}


void LocalGridMap::generate_empty_map(std::vector<std::vector<int8_t> > *p_map)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    p_map->resize(m_rolling_map.x_size);
    for(uint32_t i = 0; i < m_rolling_map.x_size; i++)
    {
        (*p_map)[i].resize(m_rolling_map.y_size);
    }
}

bool LocalGridMap::set_cost(const uint32_t mx, const uint32_t my, int8_t value)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    if(mx < m_rolling_map.x_size && my < m_rolling_map.y_size)
    {
        m_rolling_map.map[mx][my] = value;
        return true;
    }
    return false;
}

int8_t LocalGridMap::cost(const uint32_t mx, const uint32_t my)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    if (mx < m_rolling_map.x_size && my < m_rolling_map.y_size)
    {
        return m_rolling_map.map[mx][my];
    }
    else
    {
        return obstacles_cost();
    }
}

void LocalGridMap::original_point(float *p_wx, float *p_wy)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    *p_wx = m_rolling_map.origin_x;
    *p_wy = m_rolling_map.origin_y;
}

void LocalGridMap::cell_size(uint32_t *p_mx, uint32_t *p_my)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    *p_mx = m_rolling_map.x_size;
    *p_my = m_rolling_map.y_size;
}

void LocalGridMap::world_size(float *p_wx, float *p_wy)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    *p_wx = m_rolling_map.x_size * m_rolling_map.resolution;
    *p_wy = m_rolling_map.y_size * m_rolling_map.resolution;
}

bool LocalGridMap::map_to_world(const uint32_t mx, const uint32_t my, float *p_wx, float *p_wy)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    return __map_to_world(mx, my, p_wx, p_wy);
//    if (mx < m_rolling_map.x_size && my < m_rolling_map.y_size)
//    {
//        *p_wx = m_rolling_map.origin_x + (mx) * m_map.resolution;
//        *p_wy = m_rolling_map.origin_y + (my) * m_map.resolution;
//        return true;
//    }
//    return false;
}

bool LocalGridMap::world_to_map(const float wx, const float wy, uint32_t *p_mx, uint32_t *p_my)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    return __world_to_map(wx, wy, p_mx, p_my);
//    if (wx < m_rolling_map.origin_x || wy < m_rolling_map.origin_y)
//        return false;

//    *p_mx = (uint32_t)((wx - m_rolling_map.origin_x) / m_map.resolution);
//    *p_my = (uint32_t)((wy - m_rolling_map.origin_y) / m_map.resolution);

//    if (*p_mx < m_rolling_map.x_size && *p_my < m_rolling_map.y_size)
//    {
//        return true;
//    }
//    return false;
}


//void LocalGridMap::insert_obstacle_map(const std::vector<std::vector<int8_t> > &map)
//{
//    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
//    m_obstacle_map = map;
//}

void LocalGridMap::set_rolling_window_size(float window_x_size, float window_y_size)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_rolling_map.resolution = m_map.resolution;

    const float world_x_size = m_incoming_map.x_size * m_incoming_map.resolution;
    const float world_y_size = m_incoming_map.y_size * m_incoming_map.resolution;
    if(window_x_size > world_x_size)
    {
        window_x_size = world_x_size;
    }
    if(window_y_size > world_y_size)
    {
        window_y_size = world_y_size;
    }

    m_rolling_window_x_size = window_x_size;
    m_rolling_window_y_size = window_y_size;
    m_rolling_map.x_size = uint32_t(window_x_size / m_rolling_map.resolution);
    m_rolling_map.y_size = uint32_t(window_y_size / m_rolling_map.resolution);

    m_obstacle_map.resize(m_rolling_map.x_size);
    m_empty_map.resize(m_rolling_map.x_size);
    for(uint32_t i = 0; i < m_rolling_map.x_size; i++)
    {
        m_obstacle_map[i].assign(m_rolling_map.y_size, 0);
        m_empty_map[i].assign(m_rolling_map.y_size, 0);
    }
    m_rolling_map.map.resize(m_rolling_map.x_size);
    for(uint32_t i = 0; i < m_rolling_map.x_size; i++)
    {
        m_rolling_map.map[i].resize(m_rolling_map.y_size);
    }
}

inline bool LocalGridMap::__map_to_world(const uint32_t mx, const uint32_t my, float *p_wx, float *p_wy)
{
    if (mx < m_rolling_map.x_size && my < m_rolling_map.y_size)
    {
        *p_wx = m_rolling_map.origin_x + (mx) * m_map.resolution;
        *p_wy = m_rolling_map.origin_y + (my) * m_map.resolution;
        return true;
    }
    return false;
}

inline bool LocalGridMap::__world_to_map(const float wx, const float wy, uint32_t *p_mx, uint32_t *p_my)
{
    *p_mx = (uint32_t)((wx - m_rolling_map.origin_x) / m_map.resolution);
    *p_my = (uint32_t)((wy - m_rolling_map.origin_y) / m_map.resolution);

    if (*p_mx < m_rolling_map.x_size && *p_my < m_rolling_map.y_size)
    {
        return true;
    }
    return false;
}
}
