#pragma once

namespace bz_robot
{
namespace frenet_polynomial_planner
{
class Order3rdPolynomial
{
private:
    /* data */
public:
    Order3rdPolynomial(const float &xs, const float &vxs, const float &axs,
                        const float &vxe, const float &axe, const float &t);
    ~Order3rdPolynomial();
    float calc_point(const float &t);
    float calc_first_derivative(const float &t);
    float calc_second_derivative(const float &t);
    float calc_third_derivative(const float &t);
public:
    float a0;
    float a1;
    float a2;
    float a3;
    float a4;
};

}
}
