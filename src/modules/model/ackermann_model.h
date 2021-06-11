#pragma once


#include <vector>
#include <string>
#include "common/geometry.h"


namespace bz_robot
{


enum class Direct : int
{
    DIRECT_INIT = 0,    
    DIRECT_POS = 1,     //正向运动
    DIRECT_NEG = 2      //反向运动
};

enum class TurnDirect
{
    LEFT = -1,
    STRIGHT = 0,
    RIGHT = 1
};

class AckermannModel
{
public:
    AckermannModel();
    void init();
    void reset();
//    AkermanVehicle* update(const float &update_s);
    void write_params_to_file(std::string file_path);
    void read_params_from_file(std::string file_path);
    Pose<float> update(Pose<float> start, float dt, const float &linear_velocity, const float &steer_angle);
    static bool is_in_reverse_state(const float &yaw, const float &path_angle);
    static bool calc_move_directions(const std::vector<Pose<float>> &path, std::vector<Direct> *p_direct_list);
public:
    float wheel_circumference;    //轮子周长
    float wheel_base;             //轴距
    float max_steering_angle;      //前轮最大转角
    float max_linear_velocity;     //
    float max_angular_velocity;
    float max_linear_acc;
    float max_angular_acc;
    float front_axel_to_center;
    float width;
    float length;
    float center_to_front;
    float center_to_back;
    float center_to_left;
    float center_to_right;

};

}
