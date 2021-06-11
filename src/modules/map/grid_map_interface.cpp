#include "grid_map_interface.h"
#include <cassert>
#include "common/time_keeper.h"

namespace bz_robot
{
GridMapInterface::GridMapInterface(const GridMapData &map_data)
{
    m_map_data = map_data;
}


GridMapInterface::~GridMapInterface()
{
}

inline std::vector<std::vector<int8_t> > GridMapInterface::map_data()
{
    return m_map_data.map;
}


inline void GridMapInterface::update()
{
    return;
}

inline uint8_t GridMapInterface::cost(const uint32_t &mx, const uint32_t &my)
{
    if(mx < m_map_data.x_size && my < m_map_data.y_size)
    {
        return m_map_data.map[mx][my];
    }
    return obstacles_cost();
}

inline bool GridMapInterface::set_cost(const uint32_t &mx, const uint32_t &my, const uint8_t &cost)
{
    //PRINT_ERROR("should not call this func");
    //return false;
    if(mx < m_map_data.x_size && my < m_map_data.y_size)
    {
        m_map_data.map[mx][my] = cost;
        return true;
    }
    return false;
}

inline bool GridMapInterface::map_to_world(const uint32_t &mx, const uint32_t &my, float *wx, float *wy)
{
    if (mx < m_map_data.x_size && my < m_map_data.y_size)
    {
        *wx = m_map_data.origin_x + (mx) * m_map_data.resolution;
        *wy = m_map_data.origin_y + (my) * m_map_data.resolution;
        return true;
    }
    return false;
}

inline bool GridMapInterface::world_to_map(const float &wx, const float &wy, uint32_t *mx, uint32_t *my)
{
    //return mp_map->world_to_map(wx, wy, mx, my);    if (wx < m_incoming_map.origin_x || wy < m_incoming_map.origin_y)
//    return false;

    *mx = (uint32_t)((wx - m_map_data.origin_x) / m_map_data.resolution);
    *my = (uint32_t)((wy - m_map_data.origin_y) / m_map_data.resolution);

    if (*mx < m_map_data.x_size && *my < m_map_data.y_size)
    {
        return true;
    }
    return false;
}

inline bool GridMapInterface::world_to_map(const float &wx, const float &wy, float *mx, float *my)
{
    *mx = ((wx - m_map_data.origin_x) / m_map_data.resolution);
    *my = ((wy - m_map_data.origin_y) / m_map_data.resolution);

    if (*mx < m_map_data.x_size && *my < m_map_data.y_size)
    {
        return true;
    }
    return false;
}

inline uint32_t GridMapInterface::index(const uint32_t &mx, const uint32_t &my) const
{
    //return mp_map->index(mx, my);
    return m_map_data.x_size * my + mx;
}

inline void GridMapInterface::index_to_cells(uint32_t index, uint32_t *mx, uint32_t *my)
{
    //return mp_map->index_to_cells(index, mx, my);
    uint32_t x_size = m_map_data.x_size;
    uint32_t y_size = m_map_data.y_size;

    *my = index / x_size;
    *mx = index - *my * x_size;
}

inline uint32_t GridMapInterface::size_in_cells_x() const
{
//    uint32_t mx = 0;
//    uint32_t my = 0;
//    mp_map->cell_size(&mx, &my);

//    return mx;
    return m_map_data.x_size;
}

inline uint32_t GridMapInterface::size_in_cells_y() const
{
//    uint32_t mx = 0;
//    uint32_t my = 0;
//    mp_map->cell_size(&mx, &my);
//    return my;
    return m_map_data.y_size;
}

inline float GridMapInterface::size_in_meters_x() const
{
//    float wx = 0;
//    float wy = 0;
//    mp_map->world_size(&wx, &wy);
//    return wx;
    return m_map_data.x_size * m_map_data.resolution;
}

inline float GridMapInterface::size_in_meters_y() const
{
//    float wx = 0;
//    float wy = 0;
//    mp_map->world_size(&wx, &wy);
//    return wy;
    return m_map_data.y_size * m_map_data.resolution;
}

inline float GridMapInterface::origin_x() const
{
//    float x = 0;
//    float y = 0;
//    mp_map->original_point(&x, &y);
//    return x;
    return m_map_data.origin_x;
}

inline float GridMapInterface::origin_y() const
{
//    float x = 0;
//    float y = 0;
//    mp_map->original_point(&x, &y);
//    return y;
    return m_map_data.origin_y;
}

inline float GridMapInterface::resolution() const
{
//    return mp_map->resolution();
    return m_map_data.resolution;
}

inline std::vector<VectorX2<float> > GridMapInterface::robot_footprint()
{
    return m_map_data.robot_contour;
}

inline int8_t GridMapInterface::obstacles_cost()
{
    return m_map_data.obstacles_cost;
}

inline int8_t GridMapInterface::no_information_cost()
{
    return m_map_data.no_information_cost;
}
}

