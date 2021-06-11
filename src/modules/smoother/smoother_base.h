#pragma once

#include <stdint.h>
#include <iostream>
#include <memory>
#include <vector>
#include <atomic>
#include "common/geometry.h"
#include "common/data_types.h"

namespace bz_robot
{

class MapBase;
class SmootherBase
{
public:
    SmootherBase(){}
    virtual ~SmootherBase(){};
    virtual bool init(){return true;}
    virtual bool import_config(const char* config_file){return true;}
    virtual bool smooth(std::shared_ptr<MapBase> p_map, const PathData & ref_path) = 0;
    virtual PathData smoothed_path() = 0;

// protected:
//     std::vector<Pose<float>> m_ref_path;
//     std::atomic<bool> m_flag_stop;
};
}
