#include "frenet_polynomial_planner/cubic_spline_2d.h"
//#include "utils.h"
#include <math.h>
#include <algorithm>
#include <numeric>
#include <vector>

//using namespace std;
namespace bz_robot
{
namespace frenet_polynomial_planner
{
// Default constructor
CubicSpline2D::CubicSpline2D() = default;

// Construct the 2-dimensional cubic spline
CubicSpline2D::CubicSpline2D(const std::vector<float> &x,
                             const std::vector<float> &y)
{
//    std::vector<std::vector<float>> filtered_points = remove_collinear_points(x, y);
//    calc_s(filtered_points[0], filtered_points[1]);

    calc_s(x, y);
    sx = CubicSpline1D(s, x);
    sy = CubicSpline1D(s, y);
}

// Calculate the s values for interpolation given x, y
void CubicSpline2D::calc_s(const std::vector<float> &x,
                           const std::vector<float> &y)
{
    int nx = x.size();
//    std::vector<float> dx(nx);
//    std::vector<float> dy(nx);
//    adjacent_difference(x.begin(), x.end(), dx.begin());
//    adjacent_difference(y.begin(), y.end(), dy.begin());
//    dx.erase(dx.begin());
//    dy.erase(dy.begin());

//    float cum_sum = 0.0;
//    s.push_back(cum_sum);

//    for(int i = 0; i < nx - 1; i++)
//    {
//        cum_sum += hypot(dx[i], dy[i]);
//        s.push_back(cum_sum);
//    }
    //s.erase(unique(s.begin(), s.end()), s.end());

    std::vector<float> dx;
    std::vector<float> dy;
    s.resize(nx);
    if(nx > 1)
    {
        dx.resize(nx - 1);
        dy.resize(nx - 1);
    }
    float cum_sum = 0.0;
    s[0] = cum_sum;
    for(int i = 1; i < nx; ++i)
    {
        dx[i-1] = x[i] - x[i-1];
        dy[i-1] = y[i] - y[i-1];
        cum_sum += hypot(dx[i-1], dy[i-1]);
        s[i] = cum_sum;
    }
}

// Calculate the x position along the spline at given t
float CubicSpline2D::calc_x(float t)
{
    return sx.calc_der0(t);
}

// Calculate the y position along the spline at given t
float CubicSpline2D::calc_y(float t)
{
    return sy.calc_der0(t);
}

// Calculate the curvature along the spline at given t
float CubicSpline2D::calc_curvature(float t)
{
    float dx = sx.calc_der1(t);
    float ddx = sx.calc_der2(t);
    float dy = sy.calc_der1(t);
    float ddy = sy.calc_der2(t);
    float k = (ddy * dx - ddx * dy) /
               pow(pow(dx, 2) + pow(dy, 2), 1.5);
    return k;
}

// Calculate the yaw along the spline at given t
float CubicSpline2D::calc_yaw(float t)
{
    float dx = sx.calc_der1(t);
    float dy = sy.calc_der1(t);
    float yaw = atan2(dy, dx);
    return yaw;
}

// Given x, y positions and an initial guess s0, find the closest s value
float CubicSpline2D::find_s(float x, float y, float s0)
{
    float s_closest = s0;
    float closest = INFINITY;
    float si = s.front();

    do
    {
        float px = calc_x(si);
        float py = calc_y(si);
        float dist = hypot(x - px, y - py);

        if(dist < closest)
        {
            closest = dist;
            s_closest = si;
        }

        si += 0.1;
    }
    while(si < s.back());

    return s_closest;
}

// Remove any collinear points from given list of points by the triangle rule
std::vector<std::vector<float>>CubicSpline2D::remove_collinear_points(std::vector<float> x, std::vector<float> y)
{
    std::vector<std::vector<float>> filtered_points;
    std::vector<float> x_, y_;
    x_.push_back(x[0]);
    x_.push_back(x[1]);
    y_.push_back(y[0]);
    y_.push_back(y[1]);

    for(size_t i = 2; i < x.size() - 1; i++)
    {
        bool collinear = are_collinear(
                             x[i - 2], y[i - 2],
                             x[i - 1], y[i - 1],
                             x[i], y[i]
                         );

        if(collinear)
        {
            continue;
        }

        x_.push_back(x[i]);
        y_.push_back(y[i]);
    }

    // make sure to add the last point in case all points are collinear
    x_.push_back(x.back());
    y_.push_back(y.back());
    filtered_points.push_back(x_);
    filtered_points.push_back(y_);
    return filtered_points;
}

// Determine if 3 points are collinear using the triangle area rule
bool CubicSpline2D::are_collinear(float x1, float y1, float x2, float y2,
                                  float x3, float y3)
{
    float a = x1 * (y2 - y3) +
               x2 * (y3 - y1) +
               x3 * (y1 - y2);
    return a <= 0.01;
}
}
}
