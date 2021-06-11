#include <stdio.h>
#include <math.h>
#include <limits>
#include "pure_pursuit.h"
#include "modules/model/akermann_model.h"
#include "common/common.h"
#include "common/print.h"
#include "common/spdlog/spdlog.h"


using namespace std;


PurePursuit::PurePursuit():
    k(0.2),
    look_ahead_distance(1.5),
    m_start_index(0),
    m_last_start_index(0),
    m_k_p(1.0),
    m_max_iterations(50)
{

}

int PurePursuit::calc_target_index(const AkermannModel model,
                                   const Pose<float> &cur_pose,
                                   const float &cur_velocity)
{
    int len = this->way_points.size();
    float dx = 0;
    float dy = 0;
    int target_index = -1;
    if(len <= 0)
    {
        ;
    }
    else if(len == 1)
    {
        target_index = 0;
    }
    else
    {
        target_index = 0;
        float min_distance = numeric_limits<float>::max();

        // int min_index = 0;
        for(target_index = m_start_index; target_index < len; target_index++)
        {
            dx = cur_pose.position.x - this->way_points[target_index].position.x;
            dy = cur_pose.position.y - this->way_points[target_index].position.y;

            float distance = pow(dx, 2) + pow(dy, 2);
            if(distance > min_distance)
            {
                break;
            }
            else
            {
                min_distance = distance;
                m_start_index = target_index;
            }
        }
        //printf("start index = {:d} pos({:.1f}, {:.1f})\n", m_start_index, this->way_points[m_start_index].x, this->way_points[m_start_index].y);
        /**
         * 在上一步寻找与当前点最近的点中，这个点可能是即将到达的点，也可能是已经经过的点
         * 这边通过计算向量的角度，同意调整为已经经过的点
         */
        float vector_s_x = 0;
        float vector_s_y = 0;
        if(m_start_index > 0)
        {
            vector_s_x = this->way_points[m_start_index].position.x - this->way_points[m_start_index-1].position.x;
            vector_s_y = this->way_points[m_start_index].position.y - this->way_points[m_start_index-1].position.y;
            float cur_x = cur_pose.position.x - this->way_points[m_start_index].position.x;
            float cur_y = cur_pose.position.y - this->way_points[m_start_index].position.y;
            if(vector_s_x * cur_x + vector_s_y * cur_y < 0)
            {
                --m_start_index;
                if(m_start_index < m_last_start_index)
                {
                    m_start_index = m_last_start_index;
                }
            }
        }
//        //调整为即将到达的点
        ++m_start_index;
        target_index = m_start_index;
        m_last_start_index = m_start_index;
//        printf("start index = {:d} pos({:.3f}, {:.3f})\n", m_start_index,
//               this->way_points[m_start_index].position.x, this->way_points[m_start_index].position.y);
#if 0
        if(m_start_index < this->way_points.size() - 1)
        {
            float yaw = this->way_points[m_start_index].yaw;
            if(0.0 == yaw)
            {
                yaw = 0.0000000001;
            }
            float k1 = -1.0 / tan(yaw);
            float a1 = -k1;
            float b1 = 1.0;
            float c1 = -b1 * this->way_points[m_start_index].y - a1 * this->way_points[m_start_index].x;
            yaw = this->way_points[m_start_index+1].yaw;
            if(0.0 == yaw)
            {
                yaw = 0.0000000001;
            }
            float k2 = -1.0 / tan(this->way_points[m_start_index+1].yaw);
            float a2 = -k2;
            float b2 = 1.0;
            float c2 = -b2 * this->way_points[m_start_index+1].y - a2 * this->way_points[m_start_index+1].x;

            float r_x = (c1*b2-c2*b1)/(a2*b1-a1*b2);
            float r_y = (a2*c1-a1*c2)/(a1*b2-a2*b1);

            float r = hypot(r_x - this->way_points[m_start_index].x,
                             r_y - this->way_points[m_start_index].y);
            float path_curvature = 1.0/ r;
            const float max_look_ahead_distance = 10.0 * vehicle.max_linear_velocity;
        }
#endif
#if 1
            if(m_start_index < len - 1)
            {
                vector_s_x = this->way_points[m_start_index+1].position.x - this->way_points[m_start_index].position.x;
                vector_s_y = this->way_points[m_start_index+1].position.y - this->way_points[m_start_index].position.y;
            }
            float l = 0.0;
            //const float lfc = this->k * fabs(vehicle.linear_velocity) + this->look_ahead_distance;
            float lfc = m_dt * fabs(cur_velocity) + this->look_ahead_distance;
            lfc = std::max(lfc, 2.5f);
            float vector_t_x = 0;
            float vector_t_y = 0;
            while(lfc > l && (target_index + 1) < len)
            {
                dx = this->way_points[target_index + 1].position.x - this->way_points[target_index].position.x;
                dy = this->way_points[target_index + 1].position.y - this->way_points[target_index].position.y;

                l += hypot(dx, dy);
                ++target_index;
                if(target_index > 0)
                {
                    vector_t_x = this->way_points[target_index].position.x - this->way_points[target_index-1].position.x;
                    vector_t_y = this->way_points[target_index].position.y - this->way_points[target_index-1].position.y;
                }
                if(vector_s_x * vector_t_x + vector_s_y * vector_t_y < 0)
                {
                    target_index -= 1;
                    break;
                }
            }
#else
        if(m_start_index < len - 1)
        {
            vector_s_x = this->way_points[m_start_index+1].x - this->way_points[m_start_index].x;
            vector_s_y = this->way_points[m_start_index+1].y - this->way_points[m_start_index].y;
        }
        float vector_t_x = 0;
        float vector_t_y = 0;
        while(target_index + 1 < len)
        {
            ++target_index;
            if(target_index > 0)
            {
                vector_t_x = this->way_points[target_index].x - this->way_points[target_index-1].x;
                vector_t_y = this->way_points[target_index].y - this->way_points[target_index-1].y;
            }
            if(vector_s_x * vector_t_x + vector_s_y * vector_t_y < 0)
            {
                target_index -= 1;
                break;
            }
            else
            {
                vector_s_x = vector_t_x;
                vector_s_y = vector_t_y;
            }
        }


        //get target_index

#endif
        //*target_pose = this->way_points[target_index];
    }
//    printf("target index = {:d} pos({:.3f}, {:.3f})\n", target_index,
//           this->way_points[target_index].position.x, this->way_points[target_index].position.y);
    return target_index;
}

int PurePursuit::pure_pursuit_control(const float dt, AkermannModel model, Pose<float> *p_cur_pose,
                                        float *p_velocity, float *p_steer_angle)
{
    this->look_ahead_distance = model.wheel_base / tan(model.max_steering_angle) * 1.1;
    //printf("%s\n",__FUNCTION__);
//    printf("state = {:.1f} {:.1f}\n", p_cur_pose->position.x, p_cur_pose->position.y);
    m_dt = dt;
    size_t target_index = this->calc_target_index(model, *p_cur_pose, *p_velocity);

    const int length = this->way_points.size();

    if(target_index >= length)
    {
        target_index = length - 1;
    }
    Pose<float> pose = this->way_points[target_index];

    float alpha = atan2(pose.position.y - p_cur_pose->position.y, pose.position.x - p_cur_pose->position.x)
                   - p_cur_pose->heading_angle;
    float sign = 1.0;
    float expect_v = model.max_linear_velocity;
    if(m_start_index < length - 1)
    {
//        float dx = 0;
//        float dy = 0;
//        if(target_index > m_start_index)
//        {
//            dx = this->way_points[target_index].x - this->way_points[m_start_index].x;
//            dy = this->way_points[target_index].y - this->way_points[m_start_index].y;
//        }
//        else
//        {
//            dx = this->way_points[m_start_index+1].x - this->way_points[m_start_index].x;
//            dy = this->way_points[m_start_index+1].y - this->way_points[m_start_index].y;
//        }
//        float path_angle = atan2(dy, dx);
//        float angle_diff = constraint_angle_r(path_angle - model.vehicle_angle_yaw, -M_PI, M_PI);
//        if(fabs(angle_diff) > M_PI_2)
//        {
//            sign = -1.0;
//        }
        //printf("target_index = {:d}, target pose({:.1f}, {:.1f}, {:.1f})\n", target_index,
//               this->way_points[target_index].x, this->way_points[target_index].y,
//               this->way_points[m_start_index+1].yaw * 180 * M_1_PI);
        if(m_direction_list[target_index] == Direct::DIRECT_NEG)
        {
            sign = -1.0;
        }

#if 1
        float yaw1 = this->way_points[m_start_index].heading_angle;
        yaw1 = constraint_angle_r(yaw1);
        if(0.0 == yaw1)
        {
            yaw1 = 0.001;
        }
        float k1 = -1.0 / tan(yaw1);
        float a1 = -k1;
        float b1 = 1.0;
        float c1 = -b1 * this->way_points[m_start_index].position.y - a1 * this->way_points[m_start_index].position.x;

        float yaw2 = this->way_points[m_start_index+1].heading_angle;
        yaw2 = constraint_angle_r(yaw2);
        if(0.0 == yaw2)
        {
            yaw2 = 0.001;
        }
        float k2 = -1.0 / tan(yaw2);
        float a2 = -k2;
        float b2 = 1.0;
        float c2 = -b2 * this->way_points[m_start_index+1].position.y
                    - a2 * this->way_points[m_start_index+1].position.x;

        float r_x = (c1*b2-c2*b1)/(a2*b1-a1*b2);
        float r_y = (a2*c1-a1*c2)/(a1*b2-a2*b1);

        float r = hypot(r_x - this->way_points[m_start_index].position.x,
                         r_y - this->way_points[m_start_index].position.y);
        if(round(yaw1 * 1000) == round(yaw2 * 1000))
        {
            r = 1000000;
        }
//        float path_curvature = 1.0/ r;
        r = std::min(r, 100000.0f);
        const float min_r = model.wheel_base / tan(model.max_steering_angle) * 1.1;
        r = std::max(r, min_r);
        const float ratio = 1.0;
        expect_v = (1.0 / (1.0 + ratio * exp(-pow((r-min_r), 2)))) * model.max_linear_velocity;
//        PRINT_INFO("-pow((%f - %f), 2) = %f", r, min_r, -pow((r-min_r), 2));
//        PRINT_INFO("(1.0 / (1.0 + ratio * exp(-pow((r-min_r), 2))) = %f", (1.0 / (1.0 + ratio * exp(-pow((r-min_r), 2)))) );
        if(sign < 0)
        {
            expect_v = expect_v * 0.6;
        }
#endif
    }

    //printf("sign = {:.1f}\n", sign);
    const float lf = fabs(*p_velocity) * this->k + this->look_ahead_distance;
    float steering_angle = atan2(2.0 * model.wheel_base * sin(alpha) / lf, 1.0);
    steering_angle = constraint_angle_r(steering_angle, -M_PI, M_PI);
    steering_angle = std::min(steering_angle, model.max_steering_angle);
    steering_angle = std::max(steering_angle, -model.max_steering_angle);
// new add
#if 0
    float current_steering_angle = model.steering_angle;
    float angle_diff = constraint_angle_r(steering_angle - current_steering_angle, -M_PI, M_PI);

    // 假设机器人速度为0时，0.1s 前轮能够转20度，
    // 当机器人速度增大到1s时，1s钟前轮才能转20度
    // 以此来计算机器人前轮转角速度
    float angle_diff_rate = model.max_steering_angle  * exp(-model.linear_velocity * model.linear_velocity);
//    printf("current_steering_angle = {:.1f}\n", current_steering_angle * 180 * M_1_PI);
//    printf("steering_angle = {:.1f}\n", steering_angle * 180 * M_1_PI);
//    printf("model.linear_velocity = {:.1f}\n", model.linear_velocity);
//    printf("angle_diff_rate = {:.1f}\n", angle_diff_rate * 180 * M_1_PI);
//    printf("angle_diff = {:.1f}\n", angle_diff * 180 * M_1_PI);
    if(angle_diff < 0)
    {
        //printf("current_steering_angle - angle_diff_rate * dt = {:.1f}\n", (current_steering_angle - angle_diff_rate * dt) * 180 * M_1_PI);
        if(current_steering_angle - angle_diff_rate * dt < steering_angle)
        {
            //steering_angle = current_steering_angle;
        }
        else
        {
            steering_angle = current_steering_angle + angle_diff_rate * dt;
        }
    }
    else
    {
        //printf("current_steering_angle - angle_diff_rate * dt = {:.1f}\n", (current_steering_angle + angle_diff_rate * dt) * 180 * M_1_PI);

        if(current_steering_angle + angle_diff_rate * dt > steering_angle)
        {
            //steering_angle = current_steering_angle;
        }
        else
        {
            steering_angle = current_steering_angle + angle_diff_rate * dt;
        }
    }
    steering_angle = std::min(steering_angle, model.max_steering_angle);
    steering_angle = std::max(steering_angle, -model.max_steering_angle);
    //printf("steering_angle = {:.1f}\n\n", steering_angle * 180 * M_1_PI);
#endif
// end of new add

    //model.steering_angle = steering_angle;
    *p_steer_angle = steering_angle;
    float acc = this->m_k_p * (sign * expect_v - *p_velocity);
#if 1
    if(acc > model.max_linear_acc * m_dt)
    {
        acc = model.max_linear_acc;
    }
    else if (acc < -model.max_linear_acc * m_dt)
    {
        acc = -model.max_linear_acc;
    }
#endif

    if(isnan(expect_v))
    {
        PRINT_ERROR("expect_v = %f", expect_v);
        assert(0);
    }
    if(isnan(*p_velocity))
    {
        PRINT_ERROR("*p_velocity = %f", *p_velocity);
        assert(0);
    }
    if(isnan(*p_steer_angle))
    {
        PRINT_ERROR("p_steer_angle = %f", *p_steer_angle);
        assert(0);
    }

    *p_velocity += acc * dt;
    *p_velocity = std::min(*p_velocity, expect_v);
    *p_velocity = std::max(*p_velocity, -expect_v);
    //PRINT_DEBUG("v = {:.3f}, acc = {:.3f}, steer = {:.1f}", *p_velocity, acc, *p_steer_angle * 180 * M_1_PI);
    *p_cur_pose = model.update(*p_cur_pose, dt, *p_velocity, *p_steer_angle);
//    PRINT_DEBUG("target_index = %lu, state = ({:.1f} {:.1f}) {:.3f}", target_index, p_cur_pose->position.x, p_cur_pose->position.y,
//           p_cur_pose->heading_angle * 180 * M_1_PI);
    return target_index;
}

uint32_t PurePursuit::get_last_pursuit_index() const
{
    return m_start_index;
}

Pose<float> PurePursuit::get_last_pose() const
{
    return m_last_pose;
}


const std::vector<ControlParam> PurePursuit::run_task(const AkermannModel model,
                                                      const float dt,
                                                      const std::vector<Pose<float> > path,
                                                      const Pose<float> cur_pose,
                                                      float cur_velocity,
                                                      float cur_steer_angle)
{
    std::vector<ControlParam> control_params;
    control_params.clear();
    this->way_points = path;
    if(this->way_points.size() > 1)
    {
        float dx = this->way_points[this->way_points.size()-1].position.x -
                    this->way_points[this->way_points.size()-2].position.x;
        float dy = this->way_points[this->way_points.size()-1].position.y -
                    this->way_points[this->way_points.size()-2].position.y;
//        if(dx == 0)
//        {
//            dx = 0.0001;
//        }
        float path_angle = atan2(dy, dx);
        float distance = (m_dt * fabs(model.max_linear_velocity) + this->look_ahead_distance) * 0.1;
        for(int i = 0; i < 15; i++)
        {
            Pose<float> pose(this->way_points[this->way_points.size()-1].position.x + distance * cos(path_angle),
                    this->way_points[this->way_points.size()-1].position.y + distance * sin(path_angle),
                    this->way_points[this->way_points.size()-1].heading_angle);
            this->way_points.emplace_back(pose);
        }
    }
    if(! AkermannModel::calc_move_directions(this->way_points, &m_direction_list))
    {
        spdlog::error("calc move direction error\n");
        return control_params;
    }
    else
    {
        //printf("calc move direction success\n");
    }

    uint32_t iterations = 0;
    int length = this->way_points.size();
    int target_index = 0;
    Pose<float> pose = cur_pose;
    float velocity = cur_velocity;
    float steer_angle = cur_steer_angle;

    while(iterations < m_max_iterations && length-1 > target_index)
    {
        if(iterations > m_max_iterations)
        {
            spdlog::debug("[%s]-[{:d}]-[%s]: too many iterations\n", __FILE__, __LINE__, __FUNCTION__);
        }
        ++iterations;
        target_index = pure_pursuit_control(dt, model, &pose, &velocity, &steer_angle);
        //PRINT_DEBUG("v:{:.1f}, steer: {:.1f}", velocity, steer_angle * M_1_PI * 180);
        //model.control_params.push_back({model.linear_velocity, model.steering_angle});
        control_params.push_back({velocity, steer_angle});
    }
    m_last_pose.position.x = pose.position.x;
    m_last_pose.position.y = pose.position.y;
    m_last_pose.heading_angle = pose.heading_angle;

    //model.control_params.push_back({0, 0});
    //control_params.push_back({0, 0});

    return control_params;
}
