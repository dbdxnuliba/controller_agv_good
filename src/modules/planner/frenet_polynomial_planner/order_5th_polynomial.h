#pragma once
#include "common/data_types.h"

namespace bz_robot
{
namespace frenet_polynomial_planner
{
class Order4thPolynomial
{
private:
    /* data */
public:
    Order4thPolynomial(const FLOAT_T &xs, const FLOAT_T &vxs, const FLOAT_T &axs,
                        const FLOAT_T &vxe, const FLOAT_T &axe, const FLOAT_T &t);
    ~Order4thPolynomial();
    FLOAT_T calc_point(const FLOAT_T &t);
    FLOAT_T calc_first_derivative(const FLOAT_T &t);
    FLOAT_T calc_second_derivative(const FLOAT_T &t);
    FLOAT_T calc_third_derivative(const FLOAT_T &t);
public:
    FLOAT_T a0;
    FLOAT_T a1;
    FLOAT_T a2;
    FLOAT_T a3;
    FLOAT_T a4;
};



// f(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4;
class Order5thPolynomial
{
public:
    Order5thPolynomial(const FLOAT_T &xs, const FLOAT_T &vxs, const FLOAT_T &axs,
                       const FLOAT_T &xe, const FLOAT_T &vxe, const FLOAT_T &axe,
                       const FLOAT_T &t);
    ~Order5thPolynomial();
    FLOAT_T calc_point(const FLOAT_T &t);
    FLOAT_T calc_first_derivative(const FLOAT_T &t);
    FLOAT_T calc_second_derivative(const FLOAT_T &t);
    FLOAT_T calc_third_derivative(const FLOAT_T &t);
public:
    FLOAT_T a0;
    FLOAT_T a1;
    FLOAT_T a2;
    FLOAT_T a3;
    FLOAT_T a4;
    FLOAT_T a5;
};
}
}
