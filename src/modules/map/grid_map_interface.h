#pragma once

#include "map_base.h"
#include "grid_map_data.h"
namespace bz_robot
{
class GridMapInterface : public MapBase
{
public:
    GridMapInterface(const GridMapData &map_data);
    ~GridMapInterface();
    inline std::string name() {return m_map_data.name;}
    inline std::vector<std::vector<int8_t>> map_data();
    inline void update();
    inline uint8_t cost(const uint32_t &mx, const uint32_t &my);
    inline bool set_cost(const uint32_t &mx, const uint32_t &my, const uint8_t &cost);
    inline bool map_to_world(const uint32_t &mx, const uint32_t &my, float *wx, float *wy);
    inline bool world_to_map(const float &wx, const float &wy, uint32_t *mx, uint32_t *my);
    inline bool world_to_map(const float &wx, const float &wy, float *mx, float *my);
    inline uint32_t index(const uint32_t &mx, const uint32_t &my) const;
    inline void index_to_cells(uint32_t index, uint32_t *mx, uint32_t *my);
//    uint8_t* map() const;
    inline uint32_t size_in_cells_x() const;
    inline uint32_t size_in_cells_y() const;
    inline float size_in_meters_x() const;
    inline float size_in_meters_y() const;
    inline float origin_x() const;
    inline float origin_y() const;
    inline float resolution() const;
//    void resize(const float &x, const float &y, const float &w, const float &h);
//    void resize(const float &w, const float &h);
    std::vector<VectorX2<float>> robot_footprint();
    inline int8_t obstacles_cost();
    inline int8_t no_information_cost();
private:
    GridMapData m_map_data;
};
}
