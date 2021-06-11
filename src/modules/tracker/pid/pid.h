#pragma once

#include <stdint.h>
#include <iostream>
#include <vector>
#include <mutex>
#include "common/geometry.h"
#include "modules/model/ackermann_model.h"
#include "modules/tracker/pid/pid_controller.h"
#include "common/data_types.h"

namespace bz_robot
{

class ControlData;
class Prune;
/*
 * thread safety
 */

class PID
{
public:
    PID();
    ~PID();
    void set_model(AckermannModel model);
    void set_pid(const float &p, const float &i, const float &d);
    void set_path(const PathData &path, const ControlData &cur_control_data);
    ControlData run_once(const Pose<float> &cur_pose, const float &cur_velocity, const float &cur_steer_angle, const float dt);
    void reset();
    Pose<float> track_pose();
private:
    void calc_path_velocity(const std::vector<Pose<float>> &path,
                            const float &cur_velocity,
                            std::vector<float> *p_velocity_list);
    float update_velocity(const Pose<float> cur_pose, const float &cur_velocity, const float &dt);
    float update_steer_angle(const Pose<float> cur_pose, const float cur_steer_angle, const float dt);
    VectorX2<float> closest_point_on_line(VectorX2<float> cur_pose, VectorX2<float> a, VectorX2<float> b);
    float calculate_cte(VectorX2<float> cur_point,
                         VectorX2<float> point_from,
                         VectorX2<float> point_to);
    bool try_change_way_target(const Pose<float> cur_pose);
    float calculate_progress(VectorX2<float> cur_point,
                              VectorX2<float> point_from,
                              VectorX2<float> point_to);
    bool is_in_change_move_direction_behavior(const float &dt, float *p_velocity, float *p_steer_angle);
    AckermannModel m_model;
    const float m_min_distance;
    std::vector<Pose<float>> m_path;
    std::vector<Direct> m_direction_list;
    std::vector<float> m_velocity_list;
    uint32_t m_target_index;
    std::recursive_mutex m_mtx;
    bool m_is_path_aviable;
    PIDController m_pid_controller;
    std::shared_ptr<Prune> mp_path_pruner;
    float m_last_steer_angle;
    Direct m_last_move_direction;
    uint32_t m_cur_wait_index;

};

}
