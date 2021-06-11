#pragma once

class QuarticPolynomial5
{
private:
    /* data */
public:
    QuarticPolynomial5(const float &xs, const float &vxs, const float &axs,
                        const float &vxe, const float &axe, const float &t);
    ~QuarticPolynomial5();
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

class QuarticPolynomial6
{
private:
    /* data */
public:
    QuarticPolynomial6(const float &xs, const float &vxs, const float &axs,
                       const float &xe, const float &vxe, const float &axe,
                       const float &t);
    ~QuarticPolynomial6();
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
    float a5;
};
