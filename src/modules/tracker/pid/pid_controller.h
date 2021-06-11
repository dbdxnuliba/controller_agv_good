#pragma once


#include <stdint.h>
#include "common/print.h"

class PIDController
{
public:
    PIDController():
        m_error_sum(0),
        m_error_old(0)
    {
        //nothing to do
    };

    ~PIDController(){};

    void set_pid(const float p, const float i, const float d)
    {
        m_p = p;
        m_i = i;
        m_d = d;
    };

    void reset()
    {
        m_error_sum = 0;
        m_error_old = 0;
    };

    float new_value(const float error, const float dt)
    {
        float alpha = 0.0;
        //P
        alpha = -m_p * error;
//        PRINT_DEBUG("P = %f", alpha);
        //I
        //The sum is the average of the last 1000 values
        //m_error_sum = add_value_to_average(m_error_sum, dt * error, 1000);
        m_error_sum += dt * error;
        alpha -= m_i * m_error_sum;
//        PRINT_DEBUG("m_error_sum = %f, I = %f", m_error_sum, alpha);
        //D
        float d_dt_cte = (error - m_error_old) / dt;
        alpha -= m_d * d_dt_cte;
//        PRINT_DEBUG("m_error_old = %f, (error - m_error_old)/dt = %f D = %f",
//                    m_error_old, d_dt_cte, alpha);
        //save for next loop
        m_error_old = error;

        return alpha;
    };
private:
    float add_value_to_average(const float old_average, const float value_to_add, uint32_t count)
    {
        float new_average = ((old_average * (float)count) + value_to_add) / ((float)count + 1);
        return new_average;
    };
private:
    float m_p;
    float m_i;
    float m_d;
    float m_error_sum;
    float m_error_old;
};
