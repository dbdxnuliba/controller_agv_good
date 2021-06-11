#pragma once

#include "smoother/smoother_base.h"

namespace bz_robot
{

class SampleOptimizingSmootherParams;

class SampleOptimizingSmoother : public SmootherBase
{
public:
    SampleOptimizingSmoother();
    ~SampleOptimizingSmoother(){}
    bool import_config(const char* config_file);
    bool smooth(std::shared_ptr<MapBase> p_map, const PathData & ref_path);
    PathData smoothed_path();
private:
    std::shared_ptr<SampleOptimizingSmootherParams> mp_params;
};

}
