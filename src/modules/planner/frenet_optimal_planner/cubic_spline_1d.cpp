#include "cubic_spline_1d.h"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace std;
using namespace Eigen;

// Default constructor
CubicSpline1D::CubicSpline1D() = default;

// Construct the 1-dimensional cubic spline.
CubicSpline1D::CubicSpline1D(const vector<float> &v1,
                             const vector<float> &v2):
    nx(v1.size()), a(v2), x(v1), y(v2)
{
    // compute elementwise difference
    vector<float> deltas(nx);
    adjacent_difference(x.begin(), x.end(), deltas.begin());
    deltas.erase(deltas.begin());

    // compute matrix a, vector b
    MatrixXf ma = MatrixXf::Zero(nx, nx);
    VectorXf vb = VectorXf::Zero(nx);
    matrix_a(deltas, ma);
    vector_b(deltas, vb);

    // solve for c and copy to attribute vector
    MatrixXf ma_inv = ma.inverse();
    VectorXf tmp_c = ma_inv * vb;
    c.resize(tmp_c.size());
    VectorXf::Map(&c[0], tmp_c.size()) = tmp_c;

//    VectorXd vc = ma.colPivHouseholderQr().solve(vb);
//    c.resize(vc.size());
//    VectorXd::Map(&c[0], vc.size()) = vc;
    b.resize(nx - 1);
    d.resize(nx - 1);
    // construct attribute b, d
    for(int i = 0; i < nx - 1; ++i)
    {
        d[i] = ((c[i + 1] - c[i]) / (3.0 * deltas[i]));
        b[i] = ((a[i + 1] - a[i]) / deltas[i] - deltas[i] *
                    (c[i + 1] + 2.0 * c[i]) / 3.0);
    }
}

// Calculate the 0th derivative evaluated at t
float CubicSpline1D::calc_der0(float t)
{
    if(t < x.front() || t > x.back())
    {
        return NAN;
    }

    int i = search_index(t) - 1;
    float dx = t - x[i];
    return a[i] + b[i] * dx + c[i] * pow(dx, 2) + d[i] * pow(dx, 3);
}

// Calculate the 1st derivative evaluated at t
float CubicSpline1D::calc_der1(float t)
{
    if(t < x.front() || t > x.back())
    {
        return NAN;
    }

    int i = search_index(t) - 1;
    float dx = t - x[i];

    return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * pow(dx, 2);
}

// Calculate the 2nd derivative evaluated at
float CubicSpline1D::calc_der2(float t)
{
    if(t < x.front() || t > x.back())
    {
        return NAN;
    }

    int i = search_index(t) - 1;
    float dx = t - x[i];

    return 2.0 * c[i] + 6.0 * d[i] * dx;
}

// Create the constants matrix a used in spline construction
void CubicSpline1D::matrix_a(const vector<float> &deltas, MatrixXf &result)
{
    result(0, 0) = 1.0;

    for(int i = 0; i < nx - 1; i++)
    {
        if(i != nx - 2)
        {
            result(i + 1, i + 1) = 2.0 * (deltas[i] + deltas[i + 1]);
        }

        result(i + 1, i) = deltas[i];
        result(i, i + 1) = deltas[i];
    }

    result(0, 1) = 0.0;
    result(nx - 1, nx - 2) = 0.0;
    result(nx - 1, nx - 1) = 1.0;
}

// Create the 1st derivative vector b used in spline construction
void CubicSpline1D::vector_b(const vector<float> &deltas, VectorXf &result)
{
    for(int i = 0; i < nx - 2; i++)
    {
        result(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / deltas[i + 1] - 3.0 *
                        (a[i + 1] - a[i]) / deltas[i];
    }
}

// Search the spline for index closest to t
int CubicSpline1D::search_index(float t)
{
    return std::upper_bound(x.begin(), x.end(), t) - x.begin();
}
