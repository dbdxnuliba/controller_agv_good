#include "contour.h"
#include <math.h>
#include <limits>
namespace bz_robot
{
Contour::Contour()
{
    //nothing to do
    return;
}

Contour::~Contour()
{
    //nothing to do
    return;
}

void Contour::set_contour(const std::vector<VectorX2<float> > &contour_list)
{
    m_contour = contour_list;
    calculate_min_and_max_distances(contour_list, &m_inscribed_radius, &m_circumscribed_radius);
}

std::vector<VectorX2<float> > Contour::contour()
{
    return m_contour;
}

float Contour::inscribed_radius() const
{
    return m_inscribed_radius;
}

float Contour::circumscribed_radius() const
{
    return m_circumscribed_radius;
}

std::vector<VectorX2<float> > Contour::transform(const float x, const float y, const float heading_angle)
{
    std::vector<VectorX2<float>> oriented_contour_list;

    float cos_heading_angle = cos(heading_angle);
    float sin_heading_angle = sin(heading_angle);
    for (unsigned int i = 0; i < m_contour.size(); ++i)
    {
        VectorX2<float> new_pt;
      new_pt.x = x + (m_contour[i].x * cos_heading_angle - m_contour[i].y * sin_heading_angle);
      new_pt.y = y + (m_contour[i].x * sin_heading_angle + m_contour[i].y * cos_heading_angle);
      oriented_contour_list.push_back(new_pt);
    }
    return oriented_contour_list;
}

void Contour::calculate_min_and_max_distances(const std::vector<VectorX2<float> > &contour_list, float *p_min_distance, float *p_max_distance)
{
    *p_min_distance = std::numeric_limits<float>::max();
    *p_max_distance = 0.0;

    if (contour_list.size() <= 2)
    {
        return;
    }

    for (unsigned int i = 0; i < contour_list.size() - 1; ++i)
    {
        // check the distance from the robot center point to the first vertex
        float vertex_dist = hypot(contour_list[i].x, contour_list[i].y);
        float edge_dist = distance_to_line(0.0, 0.0, contour_list[i].x, contour_list[i].y,
                                            contour_list[i + 1].x, contour_list[i + 1].y);
        *p_min_distance = std::min(*p_min_distance, std::min(vertex_dist, edge_dist));
        *p_max_distance = std::max(*p_max_distance, std::max(vertex_dist, edge_dist));
    }

    // we also need to do the last vertex and the first vertex
    float vertex_dist = hypot(contour_list.back().x, contour_list.back().y);
    float edge_dist = distance_to_line(0.0, 0.0, contour_list.back().x, contour_list.back().y,
                                        contour_list.front().x, contour_list.front().y);
    *p_min_distance = std::min(*p_min_distance, std::min(vertex_dist, edge_dist));
    *p_max_distance = std::max(*p_max_distance, std::max(vertex_dist, edge_dist));
}

float Contour::distance_to_line(const float &px, const float &py, const float &x0, const float &y0, const float &x1, const float &y1)
{
    float A = px - x0;
    float B = py - y0;
    float C = x1 - x0;
    float D = y1 - y0;

    float dot = A * C + B * D;
    float len_sq = C * C + D * D;
    float param = dot / len_sq;

    float xx, yy;

    if (param < 0)
    {
        xx = x0;
        yy = y0;
    }
    else if (param > 1)
    {
        xx = x1;
        yy = y1;
    }
    else
    {
        xx = x0 + param * C;
        yy = y0 + param * D;
    }

    return hypot(xx - px, yy-py);
}
}
