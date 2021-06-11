/*
 * Author: BlindZhou
 */

#pragma once

#include <math.h>
#include "geometry.h"
#include <chrono>
#include <memory>
#include "common/data_types.h"

#define __MACRO_STR(x) #x
#define MACRO_STR(s) __MACRO_STR(s)

namespace bz_robot
{
class MapBase;


inline float radian_to_degree(const float &radian)
{
    float degree = radian * 180.0 * M_1_PI;
    return degree;
}

inline float degree_to_radian(const float& degree)
{
    float radian = degree * M_PI / 180;
    return radian;
}

#if 0
// 精确到微秒
// 使用steady_clock，即从系统开机开始算时间
inline int64_t time_stamp_us()
{
    std::chrono::steady_clock::time_point tp = std::chrono::steady_clock::now();
    std::chrono::microseconds us = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    return (int64_t) us.count();
}
#else
// 精确到微秒
// 使用system_clock，即系统时间
// 用系统时间，能够和ros时间戳统一起来
inline int64_t time_stamp_us()
{
    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    std::chrono::microseconds us = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    return (int64_t) us.count();
}

#endif

struct TimeStamp
{
    int64_t sec;
    int64_t nsec;
};

inline TimeStamp time_stamp()
{
    struct timespec ts;
    // CLOCK_REALTIME
    // CLOCK_MONOTONIC
    clock_gettime(CLOCK_REALTIME, &ts);
    TimeStamp time_stamp;
    time_stamp.sec = ts.tv_sec;
    time_stamp.nsec = ts.tv_nsec;
    return time_stamp;
}

float constraint_angle_d(float angle_d, const float &min_angle=0, const float &max_angle=360);
float constraint_angle_r(float angle_r, const float &min_angle=0, const float &max_angle=2*M_PI);
//绕原点旋转
template <class T>
VectorX2<T> rotate_2d(const VectorX2<T> &point, const float &angle_r)
{
    VectorX2<T> rotated_point = point;
    rotated_point.x = point.x * cos(angle_r) - point.y * sin(angle_r);
    rotated_point.y = point.x * sin(angle_r) + point.y * cos(angle_r);
    return rotated_point;
}
// quaternions_to euler
//
float quaternions_to_roll(const float &x, const float &y, const float &z, const float &w);
float quaternions_to_pitch(const float &x, const float &y, const float &z, const float &w);
float quaternions_to_yaw(const float &x, const float &y, const float &z, const float &w);

void rpy_to_quaternions(const float &roll, const float &pitch, const float &yaw,
                        float *px, float *py, float *pz, float *pw);
void yaw_to_quaternions(const float &yaw, float *px, float *py, float *pz, float *pw);


std::vector<Pose<float>> interpolate_path(const std::vector<Pose<float>> &path,
                                           const float &unit_len);
bool is_path_available(std::shared_ptr<MapBase> p_map,
                       const std::vector<Pose<float>> &path);
bool is_goal_reached(const Pose<float> &cur_pose,
                     const Pose<float> &goal_pose,
                     float *p_distance = nullptr);

void smooth_path(std::shared_ptr<MapBase> p_map, std::vector<Pose<float> > *p_path);
bool is_transition_valid(std::shared_ptr<MapBase> p_map, VectorX2<float> from, VectorX2<float> to);

template <typename T>
T Clamp(const T value, T bound1, T bound2) {
    if (bound1 > bound2) {
        std::swap(bound1, bound2);
    }

    if (value < bound1) {
        return bound1;
    } else if (value > bound2) {
        return bound2;
    }
    return value;
}

}
