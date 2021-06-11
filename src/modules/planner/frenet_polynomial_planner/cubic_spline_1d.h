#pragma once

#include <Eigen/LU>
#include <vector>
#include <iostream>
namespace bz_robot
{
namespace frenet_polynomial_planner
{
// 1-dimensional cubic spline class.
// For technical details see: http://mathworld.wolfram.com/CubicSpline.html
class CubicSpline1D {
public:
    int nx;
    CubicSpline1D();
    CubicSpline1D(const std::vector<float>& v1, const std::vector<float>& v2);
    float calc_der0(float t);
    float calc_der1(float t);
    float calc_der2(float t);
private:
    std::vector<float> a, b, c, d, w, x, y;
    int search_index(float t);
    void matrix_a(const std::vector<float>& deltas, Eigen::MatrixXf &result);
    void vector_b(const std::vector<float>& deltas, Eigen::VectorXf& result);
};
}
}
