#pragma once


#include <stdint.h>
#include <vector>
#include <memory>
#include "geometry.h"

namespace bz_robot
{
class MapBase;

class Prune
{
public:
    Prune();
    ~Prune()
    {

    }
    void run(std::shared_ptr<MapBase> p_map, const Pose<float> &cur_pose, std::vector<Pose<float>> *p_path, float padding = 0);
    void reset();
    // < 0 means error
    int find_closest_index_in_path(const Pose<float> &cur_pose, std::vector<Pose<float> > &path);
    uint32_t get_start_index() const
    {
        return m_begin_index;
    };
private:
    uint32_t m_begin_index;
    uint32_t m_last_begin_index;
};

}
