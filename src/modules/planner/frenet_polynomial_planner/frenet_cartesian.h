#pragma once

#include <vector>
namespace bz_robot
{
namespace frenet_polynomial_planner
{
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<float> get_frenet(float x, float y, float theta,
                         std::vector<float> maps_x,
                         std::vector<float> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<float> get_xy(float s, float d, std::vector<float> maps_s,
                     std::vector<float> maps_x,
                     std::vector<float> maps_y);
}
}
