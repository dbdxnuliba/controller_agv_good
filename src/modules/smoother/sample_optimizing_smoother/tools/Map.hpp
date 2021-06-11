//
// Created by ljn on 20-2-12.
//

#ifndef PATH_OPTIMIZER_INCLUDE_TOOLS_MAP_HPP_
#define PATH_OPTIMIZER_INCLUDE_TOOLS_MAP_HPP_

#include <iostream>
#include <cassert>
#include <stdexcept>
#include "Eigen/Core"
//#include <grid_map_core/grid_map_core.hpp>
#include "map/map_base.h"
#include "gradient_descent_smoother/dynamic_voronoi.h"
namespace bz_robot
{
namespace PathOptimizationNS {

class Map {
 public:
    Map() = delete;
    explicit Map(const std::shared_ptr<MapBase> grid_map);
    double getObstacleDistance(const Eigen::Vector2d &pos);
    bool isInside(const Eigen::Vector2d &pos) const;

 private:
    //const grid_map::GridMap &maps;
     //const MapBase &maps;
     std::shared_ptr<MapBase> mp_map;
     DynamicVoronoi m_voronoi;
};
}
}
#endif //PATH_OPTIMIZER_INCLUDE_TOOLS_MAP_HPP_
