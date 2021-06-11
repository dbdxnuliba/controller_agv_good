#pragma once

#include <vector>
#include "geometry.h"
namespace bz_robot
{
class Contour
{
public:
    Contour();
    ~Contour();

    void set_contour(const std::vector<VectorX2<float>>& contour_list);
    std::vector<VectorX2<float>> contour();
    float inscribed_radius() const;
    float circumscribed_radius() const;
    std::vector<VectorX2<float>> transform(const float x, const float y, const float heading_angle);
private:
    void calculate_min_and_max_distances(const std::vector<VectorX2<float>>& contour_list,
                                         float *p_min_distance, float *p_max_distance);
    float distance_to_line(const float &px, const float &py,
                            const float &x0, const float &y0,
                            const float &x1, const float &y1);
private:
    std::vector<VectorX2<float>>  m_contour;
    float m_inscribed_radius;
    float m_circumscribed_radius;


};
}
