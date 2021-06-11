#include "order_5th_polynomial.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "common/print.h"

namespace bz_robot
{
namespace frenet_polynomial_planner
{
Order4thPolynomial::Order4thPolynomial(const FLOAT_T &xs, const FLOAT_T &vxs, const FLOAT_T &axs,
                        const FLOAT_T &vxe, const FLOAT_T &axe, const FLOAT_T &t)
{
    // f(x) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4;
    a0 = xs;
    a1 = vxs;
    a2 = axs * 0.5;
    Eigen::Matrix2d A;
    A << 3 * pow(t,2), 4 * pow(t,3),
        6 * t, 12 * pow(t,2);

    Eigen::Vector2d B(vxe - a1 - 2 * a2 * t,
                axe - 2 * a2);
    //QR分解
    Eigen::Vector2d X = A.colPivHouseholderQr().solve(B);

//    Matrix2d a_inv = A.inverse();
//    Vector2d X = a_inv * B;

    a3 = X[0];
    a4 = X[1];
}

Order4thPolynomial::~Order4thPolynomial()
{

}

FLOAT_T Order4thPolynomial::calc_point(const FLOAT_T &t)
{
    FLOAT_T xt = a0 + a1 * t + a2 * pow(t,2) + a3 * pow(t,3) + a4 * pow(t,4);
    return xt;
}

FLOAT_T Order4thPolynomial::calc_first_derivative(const FLOAT_T &t)
{
    FLOAT_T xt = a1 + 2 * a2 * t + 3 * a3 * pow(t,2) + 4 * a4 * pow(t,3);
    return xt;
}

FLOAT_T Order4thPolynomial::calc_second_derivative(const FLOAT_T &t)
{
    FLOAT_T xt = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t,2);
    return xt;
}

FLOAT_T Order4thPolynomial::calc_third_derivative(const FLOAT_T &t)
{
    FLOAT_T xt = 6 * a3 + 24 * a4 * t;
    return xt;
}

//分别为起始点和终止点的位置，速度，加速度值，以及起始位置和终止位置的时间间隔
Order5thPolynomial::Order5thPolynomial(const FLOAT_T &xs, const FLOAT_T &vxs, const FLOAT_T &axs, const FLOAT_T &xe, const FLOAT_T &vxe, const FLOAT_T &axe, const FLOAT_T &t)
{
    // f(x) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5;
    a0 = xs;
    a1 = vxs;
    a2 = axs * 0.5;
    Eigen::Matrix3f A;
    A << pow(t, 3),    pow(t, 4),      pow(t, 5),
        3 * pow(t, 2), 4 * pow(t, 3),  5 * pow(t, 4),
        6 * t,         12 * pow(t, 2), 20 * pow(t, 3);

    Eigen::Vector3f B(xe - a0 - a1 * t - a2 * pow(t,2),
                      vxe - a1 - 2 * a2 * t,
                      axe - 2 * a2);
    //QR分解
    Eigen::Vector3f X = A.colPivHouseholderQr().solve(B);

    //std::cout << "X:\n" << X << std::endl;

    Eigen::Matrix3d Ad;
    Ad << pow(t, 3), pow(t, 4), pow(t, 5),
        3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
        6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

    Eigen::Vector3d Bd(xe - a0 - a1 * t - a2 * pow(t,2),
               vxe - a1 - 2 * a2 * t,
               axe - 2 * a2);
    //QR分解
    Eigen::Vector3d Xd = Ad.colPivHouseholderQr().solve(Bd);
    //std::cout << "Xd:\n" << Xd << std::endl;


    a3 = X[0];
    a4 = X[1];
    a5 = X[2];
}

Order5thPolynomial::~Order5thPolynomial()
{

}

FLOAT_T Order5thPolynomial::calc_point(const FLOAT_T &t)
{
    FLOAT_T xt = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t,5);
    return xt;
}

FLOAT_T Order5thPolynomial::calc_first_derivative(const FLOAT_T &t)
{
    FLOAT_T xt = a1 + 2 * a2 * t + 3 * a3 * pow(t,2) + 4 * a4 * pow(t,3) + 5 * a5 * pow(t,4);
    return xt;
}

FLOAT_T Order5thPolynomial::calc_second_derivative(const FLOAT_T &t)
{
    FLOAT_T xt = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t,2) + 20 * a5 * pow(t,3);
    return xt;
}

FLOAT_T Order5thPolynomial::calc_third_derivative(const FLOAT_T &t)
{
    FLOAT_T xt = 6 * a3 + 24 * a4 * t + 60 * a5 * pow(t, 2);
    return xt;
}
}
}
