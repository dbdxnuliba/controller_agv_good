#pragma once


#include <stdint.h>
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
//#include <boost/archive/binary_iarchive.hpp> //二进制序列化
//#include <boost/archive/binary_oarchive.hpp> //二进制序列化
//#include <boost/serialization/vector.hpp> //序列化STL容器要导入
//#include "third_libs/rest_rpc/include/rest_rpc.hpp"

#include "common/geometry.h"
namespace bz_robot
{
class GridMapData
{
public:
    GridMapData():
        obstacles_cost(100),
        no_information_cost(-1)
    {
    }
    GridMapData(std::vector<std::vector<int8_t>> &i_map,
                const float &i_resolution,
                const uint32_t &i_x_size,
                const uint32_t &i_y_size,
                const float &i_origin_x,
                const float &i_origin_y):
        map(i_map),
        resolution(i_resolution),
        x_size(i_x_size),
        y_size(i_y_size),
        origin_x(i_origin_x),
        origin_y(i_origin_y),
        obstacles_cost(100),
        no_information_cost(-1)
    {

    }

//    template<class Archive>
//    void serialize(Archive & ar, const unsigned int version)
//    {
//        ar & map;
//        ar & resolution;
//        ar & x_size;
//        ar & y_size;
//        ar & origin_x;
//        ar & origin_y;
//        ar & obstacles_cost;
//        ar & no_information_cost;
//        ar & robot_contour;
//    }
public:
    std::vector<std::vector<int8_t>> map;
    float resolution;
    uint32_t x_size;
    uint32_t y_size;
    float origin_x;
    float origin_y;
    int8_t obstacles_cost;
    int8_t no_information_cost;
    std::vector<VectorX2<float> > robot_contour;
    //std::set<uint64_t> obstacles_index_set;
    std::unordered_set<uint64_t> obstacles_index_set;
    std::string name;
};


class GridMapData2
{
//public:
//    GridMapData2():
//        obstacles_cost(100),
//        no_information_cost(-1)
//            {
//            }

public:
    //std::vector<std::vector<int8_t>> map;
    float resolution;
    uint32_t x_size;
    uint32_t y_size;
    float origin_x;
    float origin_y;
    int8_t obstacles_cost;
    int8_t no_information_cost;
    //std::vector<VectorX2<float> > robot_contour;
    //MSGPACK_DEFINE(resolution, x_size, y_size, origin_x, origin_y, obstacles_cost, no_information_cost);
};

}
