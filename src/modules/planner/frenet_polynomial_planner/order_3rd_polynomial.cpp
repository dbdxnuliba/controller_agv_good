#include "order_3rd_polynomial.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "common/print.h"
using namespace Eigen;
namespace bz_robot
{
namespace frenet_polynomial_planner
{
Order3rdPolynomial::Order3rdPolynomial(const float &xs, const float &vxs, const float &axs,
                        const float &vxe, const float &axe, const float &t)
{
    a0 = xs;
    a1 = vxs;
    a2 = axs * 0.5;
    Matrix2d A;
    A << 3 * pow(t,2), 4 * pow(t,3),
        6 * t, 12 * pow(t,2);

    Vector2d B(vxe - a1 - 2 * a2 * t,
                axe - 2 * a2);
    //QR分解
    Vector2d X = A.colPivHouseholderQr().solve(B);

//    Matrix2d a_inv = A.inverse();
//    Vector2d X = a_inv * B;

    a3 = X[0];
    a4 = X[1];
}

Order3rdPolynomial::~Order3rdPolynomial()
{

}

float Order3rdPolynomial::calc_point(const float &t)
{
    float xt = a0 + a1 * t + a2 * pow(t,2) + a3 * pow(t,3) + a4 * pow(t,4);
    return xt;
}

float Order3rdPolynomial::calc_first_derivative(const float &t)
{
    float xt = a1 + 2 * a2 * t + 3 * a3 * pow(t,2) + 4 * a4 * pow(t,3);
    return xt;
}

float Order3rdPolynomial::calc_second_derivative(const float &t)
{
    float xt = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t,2);
    return xt;
}

float Order3rdPolynomial::calc_third_derivative(const float &t)
{
    float xt = 6 * a3 + 24 * a4 * t;
    return xt;
}

}
}
