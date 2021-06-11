#include <stdio.h>
#include <math.h>
#include "interpolater.h"
#include "modules/model/akermann_model.h"

Interpolater::Interpolater(const float &max_velocity, const float &acc, const float &period_sec):
    m_max_velocity(fabs(max_velocity)),
    m_acc(fabs(acc)),
    m_period_sec(period_sec)
{
    m_brake_acc = std::min(m_max_velocity / 7, m_acc);
}

std::vector<ControlParam> Interpolater::run_task(std::vector<ControlParam> control_params)
{
//     printf("%s\n", __FUNCTION__);
    std::vector<ControlParam> output_params;
//    output_params.clear();
    if(control_params.size() < 2)
    {
        return control_params;
    }
    float init_velocity = control_params[0].velocity;
    float init_steering_angle = control_params[0].steering_angle;
//    for(auto iter = control_params.begin(); iter != control_params.end(); iter++)
//    {
//     printf("input v %f\n", iter->velocity);
//    }

    //先处理刹车点，其余工作后面再考虑
    for(auto iter = control_params.begin() + 2; iter != control_params.end(); iter++)
    {
        //表示停车，那么前一个点就是刹车点
        if(iter->velocity == 0)
        {
            float last_velocity = (iter - 1)->velocity;
            float last_steering_angle = (iter - 1)->steering_angle;
            float sign = 1.0;
            if(last_velocity < 0)
            {
                sign = -1.0;
            }
            int n = ceil(fabs(last_velocity / m_brake_acc));
            int i = 1;
            for(; i < n; i++)
            {
                if(i * (i + 1)/2 * m_brake_acc * m_period_sec > fabs(last_velocity))
                {
                    break;
                }
            }
            n = i - 1;
            float v0 = (last_velocity - sign * n * (n + 1)/2 * m_brake_acc * m_period_sec) / (n+1);
            (iter - 1)->velocity = v0 + sign * n * m_brake_acc * m_period_sec;
            // printf("last_velocity %f\n", last_velocity);
            for(int i = 0; i < n; i++)
            {
                float v = v0 + sign * i * m_brake_acc * m_period_sec;
                iter = control_params.insert(iter, {v, last_steering_angle});
                // printf("v %f\n", v);
            }
        }
    }
    output_params = control_params;
//    for(auto iter = output_params.begin(); iter != output_params.end(); iter++)
//    {
//        printf("output v %f\n", iter->velocity);
//    }
    return output_params;
}
