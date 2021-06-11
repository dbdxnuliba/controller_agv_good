#include <stdio.h>
#include <math.h>
#include <fstream>
#include <iomanip>
#include "ackermann_model.h"
#include "common/json.hpp"
#include "common/common.h"
#include "print.h"

namespace bz_robot
{


AckermannModel::AckermannModel()
{
    this->init();
    //read_params_from_file("akerman_params.json");
}

void AckermannModel::init()
{
    this->wheel_circumference = 1;
    this->wheel_base = 0.64;
//    this->steering_angle = 0;
//    this->heading_angle = 0;
    this->max_steering_angle = 20 * M_PI / 180;
    this->max_linear_velocity = 1.2;
    this->max_angular_velocity = 0.3;
    this->max_linear_acc = 0.4;
    this->max_angular_acc = 1;
    this->front_axel_to_center = 0.3;
    this->width = 0.7;
    this->length = 1.0;
    this->center_to_front = 0.5;
    this->center_to_back = 0.5;
    this->center_to_left = 0.38;
    this->center_to_right = 0.38;
}



void AckermannModel::write_params_to_file(std::string file_path)
{
    nlohmann::json j;

    j["wheel_circumference"] = this->wheel_circumference;
    j["wheel_base"] = this->wheel_base;
//    j["steering_angle"] = this->steering_angle;

//    j["heading_angle"] = this->heading_angle;
    j["max_steering_angle"] = this->max_steering_angle;
    j["max_linear_velocity"] = this->max_linear_velocity;
    j["max_angular_velocity"] = this->max_angular_velocity;
    j["max_linear_acc"] = this->max_linear_acc;
    j["max_angular_acc"] = this->max_angular_acc;
    j["front_axel_to_center"] = this->front_axel_to_center;
    std::ofstream o(file_path);
    o << std::setw(4) << j << std::endl;
}

void AckermannModel::read_params_from_file(std::string file_path)
{
    std::ifstream i(file_path);
    if(i)
    {
        nlohmann::json j;
        i >> j;
        this->wheel_circumference = j["wheel_circumference"];
        this->wheel_base = j["wheel_base"];
//        this->steering_angle = j["steering_angle"];
//        this->heading_angle = j["vehicle_angle_yaw"];
        this->max_steering_angle = j["max_steering_angle"];
        this->max_linear_velocity = j["max_linear_velocity"];
        this->max_angular_velocity = j["max_angular_velocity"];
        this->max_linear_acc = j["max_linear_acc"];
        this->max_angular_acc = j["max_angular_acc"];
        this->front_axel_to_center = j["front_axel_to_center"];
    }
    else
    {
        this->init();
        this->write_params_to_file(file_path);
    }
    
}

Pose<float> AckermannModel::update(Pose<float> start, float dt, const float &linear_velocity, const float &steer_angle)
{
    Pose<float> goal;
    float beta = 0;
//    if(linear_velocity > 0)
//    {
//        beta = atan(0.5 * tan(steer_angle));
//    }
//    else
//    {
//        beta = atan(-0.5 * tan(steer_angle));
//    }
    beta = atan(0.5 * tan(steer_angle));
    goal.position.x = start.position.x + linear_velocity * fabs(cos(beta)) * cos(beta + start.heading_angle) * dt;
    goal.position.y = start.position.y + linear_velocity * fabs(cos(beta)) * sin(beta + start.heading_angle) * dt;
    goal.heading_angle = start.heading_angle + linear_velocity * fabs(cos(beta)) * tan(steer_angle) * dt / this->wheel_base;
    goal.heading_angle  = constraint_angle_r(goal.heading_angle);
    //PRINT_DEBUG("{:.3f}, {:.3f}, {:.3f}", goal.position.x, goal.position.y, goal.heading_angle * 180 * M_1_PI);
    return goal;
}

bool AckermannModel::is_in_reverse_state(const float &yaw, const float &path_angle)
{
    bool reverse_flag = false;
    float inclined_angle = path_angle - yaw; // 范围[0, 2pi]
    inclined_angle = constraint_angle_r(inclined_angle, -M_PI, M_PI);
    //PRINT_DEBUG("heading = %f, path_angle = %f, inclined_angle = %f", radian_to_degree(yaw), radian_to_degree(path_angle), radian_to_degree(inclined_angle));
    if(fabs(inclined_angle) > M_PI_2)
    {
        reverse_flag = true;
    }
    return reverse_flag;
}

bool AckermannModel::calc_move_directions(const std::vector<Pose<float> > &path, std::vector<Direct> *p_direct_list)
{
    p_direct_list->clear();
    uint32_t len = path.size();
    if(len < 2)
    {
        printf("path len < 2\n");
        return false;
    }
    else
    {
        int index = len - 1;
        const float dx = path[index].position.x - path[index-1].position.x;
        const float dy = path[index].position.y - path[index-1].position.y;
        bool last_reverse_state = is_in_reverse_state(path[index].heading_angle, atan2(dy, dx));
        p_direct_list->emplace_back(last_reverse_state? Direct::DIRECT_NEG: Direct::DIRECT_POS);
        float last_vector_x = dx;
        float last_vector_y = dy;
        --index;
//        PRINT_DEBUG("rederfence point dx: {:.1f}, dy: {:.1f}, path_angle: {:.1f}  heading: {:.1f}->{:.1f}\n",
//               dx, dy, atan2(dy, dx) * 180 * M_1_PI,
//               path[index-1].heading_angle * 180 * M_1_PI,
//               path[index].heading_angle * 180 * M_1_PI);

        while (index > 0)
        {
            --index;
            const float dx = path[index+1].position.x - path[index].position.x;
            const float dy = path[index+1].position.y - path[index].position.y;
            if(last_vector_x * dx + last_vector_y * dy < 0)
            {
                last_reverse_state = !last_reverse_state;
            }
            last_vector_x = dx;
            last_vector_y = dy;
            p_direct_list->insert(p_direct_list->begin(), last_reverse_state? Direct::DIRECT_NEG: Direct::DIRECT_POS);
        }
        p_direct_list->insert(p_direct_list->begin(), last_reverse_state? Direct::DIRECT_NEG: Direct::DIRECT_POS);
    }

//    for(int i = 0; i < p_direct_list->size(); i++)
//    {
//        PRINT_DEBUG("({:.3f}, {:.3f}, {:.3f}) direction: {:d}\n", path[i].position.x, path[i].position.y, path[i].heading_angle * 180 * M_1_PI, (int)(*p_direct_list)[i] );
//    }
    return true;
}

}
