//
// Created by yangt on 19-5-8.
//

#ifndef COLLOSION_CHECKER_HPP
#define COLLOSION_CHECKER_HPP

#include "Map.hpp"
#include "car_geometry.hpp"
#include "../data_struct/data_struct.hpp"

namespace bz_robot
{
namespace PathOptimizationNS {

//class Config;
class Configs;
class CollisionChecker {
public:
    CollisionChecker() = delete;
    CollisionChecker(std::shared_ptr<MapBase> in_gm, std::shared_ptr<Configs> p_configs);

    bool isSingleStateCollisionFreeImproved(const State &current);

    bool isSingleStateCollisionFree(const State &current);


private:
    Map map_;
    CarGeometry car_;
    std::shared_ptr<Configs> mp_configs;
};

}
}

#endif //COLLOSION_CHECKER_HPP
