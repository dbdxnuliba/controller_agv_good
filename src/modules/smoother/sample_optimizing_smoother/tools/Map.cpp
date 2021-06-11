//
// Created by ljn on 20-2-12.
//
//#include <glog/logging.h>
#include "sample_optimizing_smoother/tools/Map.hpp"

namespace bz_robot
{
namespace PathOptimizationNS {

Map::Map(const std::shared_ptr<MapBase> grid_map) :
    mp_map(grid_map) {
//    if (!grid_map.exists("distance")) {
//        //LOG(ERROR) << "grid map must contain 'distance' layer";
//    }
    m_voronoi.buildVoronoiFromImage(grid_map);
}

double Map::getObstacleDistance(const Eigen::Vector2d &pos) {
//    if (maps.isInside(pos)) {
//        return this->maps.atPosition("distance", pos, grid_map::InterpolationMethods::INTER_LINEAR);
//    } else {
//        return 0.0;
//    }
    uint32_t mx = 0;
    uint32_t my = 0;
    if(mp_map->world_to_map(pos[0], pos[1], &mx, &my))
    {
        return m_voronoi.getDistance(mx, my);
    }
    return 0.0;
}

bool Map::isInside(const Eigen::Vector2d &pos) const {
    //return maps.isInside(pos);
    return mp_map->is_world_position_in_map_bounds(pos[0], pos[1]);
}
}
}
