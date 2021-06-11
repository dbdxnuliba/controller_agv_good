#pragma once

#include <stdint.h>
#include <memory>
#include <vector>
#include "geometry.h"
#include "grid_map.h"

namespace bz_robot
{
class LocalGridMap: public GridMap
{
public:
    LocalGridMap();
    ~LocalGridMap();
    void update_map();
    void map_data(GridMapData *p_grid_map_data);
    void set_robot_contour(const std::vector<VectorX2<float>> contour_list);
    void generate_empty_map(std::vector<std::vector<int8_t>> *p_map);
    bool set_cost(const uint32_t mx, const uint32_t my, int8_t value);
    int8_t cost(const uint32_t mx, const uint32_t my);
    void original_point(float *p_wx, float *p_wy);
    void cell_size(uint32_t *p_mx, uint32_t *p_my);
    void world_size(float *p_wx, float *p_wy);
    bool map_to_world(const uint32_t mx, const uint32_t my, float *p_wx, float *p_wy);
    bool world_to_map(const float wx, const float wy, uint32_t *p_mx, uint32_t *p_my);
    //void insert_obstacle_map(const std::vector<std::vector<int8_t>> &map);
    void set_rolling_window_size(float window_x_size, float window_y_size);
private:
    virtual inline bool __map_to_world(const uint32_t mx, const uint32_t my, float *p_wx, float *p_wy);
    virtual inline bool __world_to_map(const float wx, const float wy, uint32_t *p_mx, uint32_t *p_my);
    void generate_rolling_window_map(float window_x_size, float window_y_size);
private:
//    int8_t** mp_rolling_map;
    GridMapData m_rolling_map;
    float m_rolling_window_x_size;
    float m_rolling_window_y_size;
};
}
