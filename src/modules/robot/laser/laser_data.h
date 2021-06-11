#pragma once

#include <vector>
#include <boost/archive/binary_iarchive.hpp> //二进制反序列化
#include <boost/archive/binary_oarchive.hpp> //二进制序列化
#include <boost/serialization/vector.hpp>
#include "common/geometry.h"

struct LaserData
{
    VectorX2<float> pos_offset;
    float angle_offset = 0;
    float start_angle = 0;
    float angle_step = 0;
    bool is_mirror = 0;
    std::vector<float> distance_list;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & pos_offset;
        ar & angle_offset;
        ar & start_angle;
        ar & angle_step;
        ar & is_mirror;
        ar & distance_list;
    }
};
