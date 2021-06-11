#pragma once

#include <stdint.h>
#include <vector>
#include <queue>
#include <deque>
#include <mutex>
#include <map>
#include "common/data_types.h"
#include "common/geometry.h"
#include "grid_map_data.h"
#include "contour.h"

namespace bz_robot
{
class CellData
{
public:
  CellData(uint32_t i=0, uint32_t i_x=0, uint32_t i_y=0, uint32_t sx=0, uint32_t sy=0) :
      index(i), x(i_x), y(i_y), src_x(sx), src_y(sy)
  {
      ;
  }
  uint32_t index;
  uint32_t x, y;
  uint32_t src_x, src_y;
};

class CellData2
{
public:
    CellData2(uint32_t i_x=0, uint32_t i_y=0, int idx=0, int idy=0) :
        x(i_x), y(i_y), dx(idx), dy(idy)
    {
        ;
    }
    uint32_t x, y;
    int dx, dy;
};


class GridMap
{
public:
    class Obstacle
    {
    public:
        int64_t time_stamp_us;
        std::string name;
        std::vector<VectorX2<FLOAT_T>> data;
    };

public:
    GridMap();
    virtual ~GridMap();
    //这个从地图文件中读取进来的，是原始数据，不可变
    //probabilities are in the range [0,100].  Unknown is -1.
    void set_incoming_map_data(const int8_t * p_incoming_map_data,
                        const float resolution,
                        const uint32_t x_size,
                        const uint32_t y_size,
                        const float origin_x,
                        const float origin_y);
    void set_incoming_map_data(const GridMapData &incoming_map);

    void set_resolution(const float resolution);
    float resolution();
    virtual void update_map();
    virtual void map_data(GridMapData *p_grid_map_data);
    virtual void set_robot_contour(const std::vector<VectorX2<float>> contour_list);
    std::vector<VectorX2<float>> robot_contour();
    virtual void generate_empty_map(std::vector<std::vector<int8_t>> *p_map);
    virtual bool set_cost(const uint32_t mx, const uint32_t my, int8_t value);
    virtual int8_t cost(const uint32_t mx, const uint32_t my);
    virtual inline uint32_t index(const uint32_t mx, const uint32_t my);
    virtual void index_to_cells(uint32_t index, uint32_t *mx, uint32_t *my);
    virtual void original_point(float *p_wx, float *p_wy);
    virtual void cell_size(uint32_t *p_mx, uint32_t *p_my);
    virtual void world_size(float *p_wx, float *p_wy);
    void set_robot_pose(const float x, const float y, const float heading_angle);
    virtual bool map_to_world(const uint32_t mx, const uint32_t my, float *p_wx, float *p_wy);
    virtual bool world_to_map(const float wx, const float wy, uint32_t *p_mx, uint32_t *p_my);
    void set_inflation_radius(const float inflaction_radius, const float drop_speed);
    virtual inline void set_static(bool is_static);
    virtual inline bool is_static();
    virtual inline void enable_inflaction(bool is_inflaction_enabled);
    virtual inline bool is_inflaction_enabled();
    virtual void insert_obstacle_map(const std::vector<std::vector<int8_t>> &map);
    virtual inline void enable_obstacle_map(bool is_obstacle_map_enabled);
    virtual inline bool is_obstacle_map_enabled();
    virtual inline void set_obstacles_cost(const int8_t value);
    virtual inline int8_t obstacles_cost();
    virtual inline bool is_obstacles(const uint32_t mx, const uint32_t my);
    virtual void set_obstacles_map_over_time(const uint32_t &over_time_ms);
    virtual inline void insert_obstacles(const Obstacle& obstacle);
protected:
    virtual inline bool __map_to_world(const uint32_t mx, const uint32_t my, float *p_wx, float *p_wy);
    virtual inline bool __world_to_map(const float wx, const float wy, uint32_t *p_mx, uint32_t *p_my);
    virtual inline void update_obstacles_map();
    void inflation_obstacles(std::vector<std::vector<int8_t>>* p_map, const uint32_t map_x_size, const uint32_t map_y_size);
    inline void enqueue(uint32_t index, uint32_t mx, uint32_t my, uint32_t src_x, uint32_t src_y, std::map<uint32_t, std::vector<CellData> > *p_inflation_cells);
protected:
    std::recursive_mutex m_mtx;
    GridMapData m_incoming_map;
    Contour m_robot_contour;
    GridMapData m_map;
    GridMapData m_original_map;
    float m_robot_x;
    float m_robot_y;
    float m_robot_heading_angle;
    uint32_t m_inflaction_cost_map_size;
    std::vector<std::vector<int8_t>> m_empty_map;
    std::vector<std::vector<int8_t>> m_obstacle_map;
    std::vector<std::vector<int8_t>> m_inflaction_cost_map;

    std::vector<std::vector<int8_t>> m_cached_costs;
    std::vector<std::vector<uint32_t>> m_cached_distances;
    std::vector<bool> m_seen;

    uint32_t m_cell_inflation_radius;
//    std::map<float, std::vector<CellData> > m_inflation_cells;
    bool m_is_static;
    bool m_is_enable_inflaction;
    bool m_is_enable_obstacle_map;
    int8_t m_obstacles_cost;
    uint32_t m_obstacles_map_over_time_ms;
    std::deque<Obstacle> m_obstacles_map_queue;
};
}
