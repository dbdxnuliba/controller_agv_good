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

class PlannerBase
{
public:
    PlannerBase(){}
    virtual ~PlannerBase(){};
    virtual bool init(){return true;}
    //virtual void init(std::shared_ptr<MapBase> p_map){};
    virtual bool import_config(const char* config_file){return true;}
    //virtual void upadte_map(std::shared_ptr<MapBase> p_map){};
    virtual void set_reference_path(const PathData & ref_path)
    {
        m_ref_path = ref_path;
    }
    virtual bool plan(std::shared_ptr<MapBase> p_map, Pose<float> start_pose,
                      Pose<float> goal_pose, PathData *p_path) = 0;
    virtual void reset(){}
    virtual void stop() {m_flag_stop = true;}
    virtual bool is_stop() {return m_flag_stop;}
protected:
    std::vector<Pose<float>> m_ref_path;
    std::atomic<bool> m_flag_stop;
};
}
