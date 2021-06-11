#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "common/common.h"
#include "modules/map/map_base.h"


namespace bz_robot
{
float constraint_angle_d(float angle_d, const float& min_angle, const float& max_angle)
{
    const float angle_it = max_angle - min_angle;
    if((u_int32_t)round(angle_it*1000) != (u_int32_t)360 * 1000)
    {
        exit(1);
    }
    //int n = angle_d / 360;
    //angle_d = angle_d - 360 * n;
    angle_d = fmod(angle_d, 360);
    while (angle_d >= max_angle)
    {
        angle_d -= angle_it;
    }
    while (angle_d < min_angle)
    {
        angle_d += angle_it;
    }
    return angle_d;
}

float constraint_angle_r(float angle_r, const float& min_angle, const float& max_angle)
{
    const float angle_it = max_angle - min_angle;
    assert(int(round(angle_it * 1000)) == int(round(M_PI * 2 * 1000)));
    //int n = angle_r * M_2_PI;
    //int n = angle_r * M_1_PI * 0.5;
    angle_r = fmod(angle_r, 2 * M_PI);
    while (angle_r >= max_angle)
    {
        angle_r -= (M_PI * 2);
    }
    while (angle_r < min_angle)
    {
        angle_r += (M_PI * 2);
    }
    return angle_r;
}

float quaternions_to_roll(const float &x, const float &y, const float &z, const float &w)
{
    return atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
}

float quaternions_to_pitch(const float &x, const float &y, const float &z, const float &w)
{
    return asin(2 * (w * y - z * x));
}

float quaternions_to_yaw(const float &x, const float &y, const float &z, const float &w)
{
    return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

void rpy_to_quaternions(const float &roll, const float &pitch, const float &yaw,
                        float *px, float *py, float *pz, float *pw)
{
    float halfYaw = yaw * 0.5;
    float halfPitch = pitch * 0.5;
    float halfRoll = roll * 0.5;
    float cosYaw = halfYaw;
    float sinYaw = halfYaw;
    float cosPitch = halfPitch;
    float sinPitch = halfPitch;
    float cosRoll = halfRoll;
    float sinRoll = halfRoll;
    *px = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; //x
    *py = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; //y
    *pz = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; //z
    *pw = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; //formerly yzx
}

void yaw_to_quaternions(const float &yaw, float *px, float *py, float *pz, float *pw)
{
    return rpy_to_quaternions(0, 0, yaw, px, py, pz, pw);
}



std::vector<Pose<float>> interpolate_path(const std::vector<Pose<float>> &path,
                                                  const float &unit_len)
{
    std::vector<Pose<float>> interpolated_path;
    size_t len = path.size();
    if(len > 1)
    {
        for(size_t i = 0; i < len-1; ++i)
        {
            //interpolated_path.emplace_back(path[i]);
            float dx = path[i+1].position.x - path[i].position.x;
            float dy = path[i+1].position.y - path[i].position.y;
            float k = atan2(dy, dx);
            float distance = hypot(dx, dy);
            int n = int(distance / unit_len);
            const float n_1 = 1.0 / n;
            for(int j = 0; j < n; ++j)
            {
                Pose<float> temp_pose;
                temp_pose.position.x = path[i].position.x + n_1 * j * dx;
                temp_pose.position.y = path[i].position.y + n_1 * j * dy;
                temp_pose.heading_angle = k;
                interpolated_path.emplace_back(temp_pose);
            }
        }
        interpolated_path.emplace_back(path[len-1]);
    }
    else
    {
        return path;
    }
    return interpolated_path;
}


bool is_path_available(std::shared_ptr<MapBase> p_map,
                       const std::vector<Pose<float>> &path)
{
    if(path.size() < 2)
    {
        return false;
    }
    bool is_path_available = true;
    const int8_t obstacles_cost = p_map->obstacles_cost() - 2;
    const std::vector<Pose<float>> &interpolateed_path = path;
//    std::vector<Pose<float>> interpolateed_path = std::move(interpolate_path(path, p_map->resolution()));
    const size_t len = interpolateed_path.size();

    for(size_t i = 0; i < len; ++i)
    {
        uint32_t map_pose_x = 0;
        uint32_t map_pose_y = 0;
        if(p_map->world_to_map(interpolateed_path[i].position.x, interpolateed_path[i].position.y,
                                &map_pose_x, &map_pose_y))
        {
            if(p_map->cost(map_pose_x, map_pose_y) > obstacles_cost)
            {
                //PRINT_WARN("exist obstacles!!");
                is_path_available = false;
                break;
            }
        }
        else
        {
            PRINT_WARN("pose out of map!!");
            is_path_available = false;
            break;
        }
    }
    return is_path_available;
}



bool is_goal_reached(const Pose<float> &cur_pose,
                     const Pose<float> &goal_pose,
                     float *p_distance)
{
    float dx = goal_pose.position.x - cur_pose.position.x;
    float dy = goal_pose.position.y - cur_pose.position.y;

    //judge is close to goal
    float delta_distance = hypot(dx, dy);

    float delta_angle = cur_pose.heading_angle - goal_pose.heading_angle;
    delta_angle = constraint_angle_r(delta_angle, -M_PI, M_PI);
    delta_angle = fabs(delta_angle);

    const float MIN_DISTANCE = 0.50;
    const float MIN_ANGLE = 20 * M_PI / 180;
    if(p_distance != nullptr)
    {
        *p_distance = delta_distance;
    }

    if(fabs(delta_distance) < fabs(MIN_DISTANCE) && delta_angle < MIN_ANGLE)
    {
        return true;
    }

    return false;
}


void smooth_path(std::shared_ptr<MapBase> p_map, std::vector<Pose<float> > *p_path)
{
    int span = 2;

    while(span < p_path->size())
    {
        bool changed = false;

        for(int i = 0; i + span < p_path->size(); i++)
        {
            if(is_transition_valid(p_map, (*p_path)[i].position, (*p_path)[i + span].position))
            {
                for(int x = 1; x < span; x++)
                {
                    p_path->erase(p_path->begin() + i + 1);
                }

                changed = true;
            }
        }

        if(!changed)
        {
            span++;
        }
    }
}

bool is_transition_valid(std::shared_ptr<MapBase> p_map, VectorX2<float> from, VectorX2<float> to)
{
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;
    p_map->world_to_map(from.x, from.y, (uint32_t *)&x1, (uint32_t *)&y1);
    p_map->world_to_map(to.x, to.y, (uint32_t *)&x2, (uint32_t *)&y2);
    int dx = x2 - x1;
    int dy = y2 - y1;
    uint32_t len = ceil(hypot(x1 - x2, y1 - y2));
    //int8_t obstacle_cost = p_map->obstacles_cost() - 10;
    int8_t obstacle_cost = 0;
    for(uint32_t i = 1; i < len; ++i)
    {
        uint32_t pose_x = x1 + 1.0 * dx * i / len;
        uint32_t pose_y = y1 + 1.0 * dy * i / len;
        if(p_map->cost(pose_x, pose_y) > obstacle_cost)
        {
            return false;
        }
    }
    return true;
}
}
