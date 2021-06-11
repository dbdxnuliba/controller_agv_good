#include "tracker/pid/pid.h"
#include "common/common.h"
#include "common/print.h"
#include "common/prune.h"
#include "common/data_types.h"

namespace bz_robot
{


PID::PID():
    m_min_distance(2),
    m_is_path_aviable(false)
{
    mp_path_pruner = std::make_shared<Prune>();
    m_pid_controller = PIDController();
//    //no acc
//    m_pid_controller.set_pid(0.7, 0.3, 0.07);
//    //acc
//    m_pid_controller.set_pid(0.7, 0.3, 0.07);
    m_last_steer_angle = 0;
    m_last_move_direction = Direct::DIRECT_INIT;
    m_cur_wait_index = 0;
}

PID::~PID()
{

}

void PID::set_model(AckermannModel model)
{
    m_model = model;
}

void PID::set_pid(const float &p, const float &i, const float &d)
{
    m_pid_controller.set_pid(p, i, d);
}

void PID::set_path(const PathData &path, const ControlData &cur_control_data)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    const FLOAT_T &cur_velocity = cur_control_data.velocity;
    m_path.clear();
    m_is_path_aviable = true;
    //m_path = path;
    m_target_index = 1;

    //m_pid_controller.reset();
    if(path.size() < 2)
    {
        m_is_path_aviable = false;
        PRINT_ERROR("m_is_path_aviable = {:d}, path.size() < 2", m_is_path_aviable);
    }
    else
    {
        //remove repeat point
        m_path.emplace_back(path[0]);
        for(size_t i = 0; i < path.size(); ++i)
        {
            if(round(m_path.back().position.x * 100) == round(path[i].position.x * 100) &&
               round(m_path.back().position.y * 100) == round(path[i].position.y * 100))
            {
                continue;
            }
            m_path.emplace_back(path[i]);
        }
#if 1
        //for test tracker, only positive move!!!
        m_direction_list.assign(m_path.size(), Direct::DIRECT_POS);
#else
        if( !AckermannModel::calc_move_directions(m_path, &m_direction_list))
        {
            m_is_path_aviable = false;
            PRINT_ERROR("m_is_path_aviable = {:d}, calc_move_directions error", m_is_path_aviable);
        }
#endif
    }
    if(m_is_path_aviable)
    {
        //PRINT_DEBUG("pid reset path pruner");
        mp_path_pruner->reset();
        calc_path_velocity(m_path, cur_velocity, &m_velocity_list);
    }
//    for(size_t i = 0; i < std::min(m_path.size(), (size_t)10); ++i)
//    {
//        PRINT_INFO("path[%lu] -  {:.3f}, {:.3f}, {:.3f}, direct {:d}",
//                   i, m_path[i].position.x, m_path[i].position.y, m_path[i].heading_angle * 180 * M_1_PI,
//                   m_direction_list[i] == Direct::DIRECT_NEG?-1:1);
//        //for test
//        if(m_direction_list[i] == Direct::DIRECT_NEG)
//        {
//            PRINT_ERROR("in backward mode\n\n\n\n");
//            exit(-2);
//        }
//    }
    //PRINT_INFO("m_is_path_aviable = {:d}", m_is_path_aviable);
}

ControlData PID::run_once(const Pose<float> &cur_pose,
                           const float &cur_velocity,
                           const float & cur_steer_angle,
                           const float dt)
{
    //std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    ControlData control_param;
    control_param.dt = dt;
    control_param.velocity = 0;
    control_param.steer_angle = 0;
    if(m_is_path_aviable)
    {
        if(try_change_way_target(cur_pose))
        {
            //if(!is_in_change_move_direction_behavior(dt, &control_param.velocity, &control_param.steering_angle))
            {
                control_param.velocity = update_velocity(cur_pose, cur_velocity, dt);
                control_param.steer_angle = update_steer_angle(cur_pose, cur_steer_angle, dt);
                //printf("fabs((1 - fabs(control_param.steering_angle/m_model.max_steering_angle) * 0.1)) = %f\n\n", fabs((1 - fabs(control_param.steering_angle/m_model.max_steering_angle) * 0.1)));
                //control_param.velocity  = control_param.velocity * fabs((1 - fabs(control_param.steering_angle/m_model.max_steering_angle) * 0.1));
            }

        }
        else
        {
            control_param.velocity = 0;
            control_param.steer_angle = 0;
            m_is_path_aviable = false;
        }
    }
    //PRINT_INFO("m_is_path_aviable = {:d}", m_is_path_aviable);
    return control_param;
}

void PID::reset()
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    m_pid_controller.reset();
    m_last_steer_angle = 0;
    m_last_move_direction = Direct::DIRECT_INIT;
    m_cur_wait_index = 0;
}

Pose<float> PID::track_pose()
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    if(m_target_index < m_path.size())
    {
        return m_path[m_target_index];
    }
    return Pose<float>();
}


void PID::calc_path_velocity(const std::vector<Pose<float> > &path,
                             const float &cur_velocity,
                             std::vector<float> *p_velocity_list)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    size_t path_size = m_path.size();
    p_velocity_list->resize(path_size);
    //const float acc = 0.25;
    for(size_t i = 0; i < path_size; ++i)
    {
        //float wanted_velocity = std::min(fabs(cur_velocity) + acc * i,m_model.max_linear_velocity);
        float wanted_velocity = m_model.max_linear_velocity;
        if(m_direction_list[i] == Direct::DIRECT_NEG)
        {
            wanted_velocity *= 0.5;
        }
        float distance_to_turning_point = 0;
        for(size_t j = i+1; j < path_size; ++j)
        {
            const float dx = path[j].position.x - path[j-1].position.x;
            const float dy = path[j].position.y - path[j-1].position.y;
            distance_to_turning_point += hypot(dx, dy);
            if(j == path_size-1 || m_direction_list[i] != m_direction_list[j])
            {
                break;
            }
        }
        if(distance_to_turning_point < m_min_distance)
        {
            wanted_velocity = (distance_to_turning_point / m_min_distance) * wanted_velocity;
            wanted_velocity = std::max(wanted_velocity, 0.1f * m_model.max_linear_velocity);
            wanted_velocity = std::min(wanted_velocity, m_model.max_linear_velocity);
        }
        (*p_velocity_list)[i] = wanted_velocity;
    }

//    for(int i = 0; i < p_velocity_list->size(); ++i)
//    {
//        PRINT_DEBUG("[{:d}]: %f", i, (*p_velocity_list)[i]);
//    }
}

float PID::update_velocity(const Pose<float> cur_pose, const float &cur_velocity, const float &dt)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    float sign = 1.0;
    float wanted_velocity = m_velocity_list[m_target_index];
    float dx = m_path[m_target_index].position.x - m_path[m_target_index-1].position.x;
    float dy = m_path[m_target_index].position.y - m_path[m_target_index-1].position.y;
    float path_angle = atan2(dy, dx);
    float angle_diff = constraint_angle_r(path_angle - cur_pose.heading_angle, -M_PI, M_PI);
    if(fabs(radian_to_degree(angle_diff)) > 5.0)
    {
        float deviate_from_path_velocity = m_model.max_linear_velocity * 0.5;
        wanted_velocity = std::min(wanted_velocity, deviate_from_path_velocity);
    }
    if(m_direction_list[m_target_index] == Direct::DIRECT_NEG)
    {
        sign = -1.0;
        wanted_velocity *= -1.0;
    }

    float next_acc = (wanted_velocity - cur_velocity)/dt;
    next_acc = std::min(next_acc, m_model.max_linear_acc);
    next_acc = std::max(next_acc, -m_model.max_linear_acc);
    float next_velocity = cur_velocity + next_acc * dt;

    //return wanted_velocity;
    //PRINT_DEBUG("wanted_velocity %f, cur_v:%f  next_v: %f", wanted_velocity, cur_velocity, next_velocity);
    return next_velocity;
}

float PID::update_steer_angle(const Pose<float> cur_pose, const float cur_steer_angle, const float dt)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
#if 0
    //pid method
    float cte = calculate_cte(cur_pose.position,
                               m_path[m_target_index-1].position,
                               m_path[m_target_index].position);
    if(m_direction_list[m_target_index] == Direct::DIRECT_NEG)
    {
        cte *= -1;
    }
    float steer_angle = m_pid_controller.new_value(cte, dt);
    PRINT_INFO("cur pose({:.3f}, {:.3f}), target({:d}): ({:.3f}, {:.3f})->({:.3f}, {:.3f})",
               cur_pose.position.x, cur_pose.position.y, m_target_index,
               m_path[m_target_index-1].position.x, m_path[m_target_index-1].position.y,
               m_path[m_target_index].position.x, m_path[m_target_index].position.y);
    PRINT_INFO("steer_angle: %f, cte: %f", steer_angle*180*M_1_PI, cte);
    steer_angle = std::min(steer_angle, m_model.max_steering_angle);
    steer_angle = std::max(steer_angle, -m_model.max_steering_angle);
    return steer_angle;
#endif
    //stanly method with pid
    float cte = calculate_cte(cur_pose.position,
                               m_path[m_target_index-1].position,
                               m_path[m_target_index].position);
    float sign = 1.0;
    if(m_direction_list[m_target_index] == Direct::DIRECT_NEG)
    {
        PRINT_INFO("backward state");
        sign = -1.0;
    }
    cte *= sign;

    float velocity = m_velocity_list[m_target_index];

    float tracker_yaw = m_path[m_target_index].heading_angle;
    const float k = 1.0;
    float theta_e = constraint_angle_r(tracker_yaw - cur_pose.heading_angle, -M_PI, M_PI);
    theta_e *= sign;

    float error_front_axle =
            m_path[m_target_index].position.x * cos(cur_pose.heading_angle + M_PI_2) +
            m_path[m_target_index].position.y * sin(cur_pose.heading_angle + M_PI_2);


    float len = m_pid_controller.new_value(cte, dt);
    float theta_d = atan2(len, velocity);
    float steer_angle = theta_d + theta_e;
    //add acc limit
    float next_acc = (steer_angle - cur_steer_angle)/dt;
    next_acc = std::min(next_acc, m_model.max_angular_acc);
    next_acc = std::max(next_acc, -m_model.max_angular_acc);
    steer_angle = cur_steer_angle + next_acc * dt;
    m_last_steer_angle = steer_angle;
    //end of acc limit

//    PRINT_INFO("cur pose({:.3f}, {:.3f}, {:.3f}), \n"
//               "target({:d}): ({:.3f}, {:.3f}, {:.3f})->({:.3f}, {:.3f}, {:.3f})",
//               cur_pose.position.x,
//               cur_pose.position.y,
//               cur_pose.heading_angle*180*M_1_PI,
//               m_target_index,
//               m_path[m_target_index-1].position.x,
//               m_path[m_target_index-1].position.y,
//               m_path[m_target_index-1].heading_angle*180*M_1_PI,
//               m_path[m_target_index].position.x,
//               m_path[m_target_index].position.y,
//               m_path[m_target_index].heading_angle*180*M_1_PI);
//    PRINT_INFO("steer_angle: %f(%f), steer_acc = %f, cte: %f, len =%f\n"
//               "error_front_axle = %f, theta_e = %f, theta_d = %f",
//               steer_angle*180*M_1_PI, steer_angle, next_acc,
//               cte, len, error_front_axle, theta_e*180*M_1_PI, theta_d);
    steer_angle = std::min(steer_angle, m_model.max_steering_angle);
    steer_angle = std::max(steer_angle, -m_model.max_steering_angle);
    return steer_angle;

}

VectorX2<float> PID::closest_point_on_line(VectorX2<float> cur_pose, VectorX2<float> a, VectorX2<float> b)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
//    PRINT_INFO("p1(%f, %f) p2(%f, %f)", a.x, a.y, b.x, b.y);
    VectorX2<float> point;
    VectorX2<float> a_p = cur_pose - a;
    VectorX2<float> a_b = b - a;
    float len_square = a_b.x * a_b.x + a_b.y * a_b.y;
    float ab_ap_product = a_b.x * a_p.x + a_b.y * a_p.y;
    float distance = ab_ap_product / len_square;
    if(distance < 0)
    {
        point = a;
    }
    else if(distance > 1)
    {
        point = b;
    }
    else
    {
        point = a_b;
        point.x *= distance;
        point.y *= distance;
        point = a + point;
    }
    return point;
}

float PID::calculate_cte(VectorX2<float> cur_point,
                          VectorX2<float> point_from,
                          VectorX2<float> point_to)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    VectorX2<float> cloest_point = closest_point_on_line(cur_point, point_from, point_to);
    const float dx = cur_point.x - cloest_point.x;
    const float dy = cur_point.y - cloest_point.y;
    float cte = hypot(dx, dy);
    //determine if CTE is negative or positive so we can steer in the direction we need
    //Is the car to the right or to the left of the upcoming waypoint
    VectorX2<float> to_cur_vec = cur_point - point_from;
    VectorX2<float> to_point_vec = point_to - point_from;
    float angle1 = atan2(to_cur_vec.y, to_cur_vec.x);
    float angle2 = atan2(to_point_vec.y, to_point_vec.x);
    float angle_diff = constraint_angle_r(angle1 - angle2, -M_PI, M_PI);
    if(angle_diff < 0)
    {
        cte *= -1.0;
    }
    return cte;
}

bool PID::try_change_way_target(const Pose<float> cur_pose)
{
    //std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    bool status = true;
    VectorX2<float> cur_point = cur_pose.position;
    VectorX2<float> point_from = m_path[m_target_index-1].position;
    VectorX2<float> point_to = m_path[m_target_index].position;
    if(calculate_progress(cur_point, point_from, point_to) > 1.0)
    {
        ++m_target_index;
        if(m_target_index > m_path.size() - 2)
        {
            m_path.clear();
            status = false;
        }
    }

    int index = mp_path_pruner->find_closest_index_in_path(cur_pose, m_path);
    if(index < 0 || index > m_path.size() - 2)
    {
        m_path.clear();
        status = false;
    }
//    Pose<float> front_axel_pose = cur_pose;
//    front_axel_pose.position.x = cur_pose.position.x + m_model.front_axel_to_center * cos(cur_pose.heading_angle);
//    front_axel_pose.position.x = cur_pose.position.y + m_model.front_axel_to_center * sin(cur_pose.heading_angle);
//    index = mp_path_pruner->find_closest_index_in_path(front_axel_pose, m_path);
//    if(index < 0 || index > m_path.size() - 2)
//    {
//        m_path.clear();
//        status = false;
//    }
    ++index;
    m_target_index = (uint32_t)index;
    return status;
}

float PID::calculate_progress(VectorX2<float> cur_point, VectorX2<float> point_from, VectorX2<float> point_to)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    float rx = cur_point.x - point_from.x;
    float ry = cur_point.y - point_from.y;
    float dx = point_to.x - point_from.x;
    float dy = point_to.y = point_from.y;
    float progress = ((rx * dx) + (ry * dy)) / ((dx * dx) + (dy * dy));
    return progress;
}

bool PID::is_in_change_move_direction_behavior(const float &dt, float *p_velocity, float *p_steer_angle)
{
    if(m_last_move_direction == Direct::DIRECT_INIT)
    {
        m_cur_wait_index = 0;
        m_last_move_direction = m_direction_list[m_target_index];
    }
    else if(m_last_move_direction != Direct::DIRECT_INIT &&
        m_last_move_direction != m_direction_list[m_target_index])
    {
        //means direction switchs
        uint32_t total_wait_index = ceil(m_model.max_steering_angle / m_model.max_angular_acc / dt * 2);
        if(m_cur_wait_index < total_wait_index)
        {
            ++m_cur_wait_index;
            *p_velocity = 0;
            if(m_last_steer_angle < 0)
            {
                *p_steer_angle = m_model.max_steering_angle;
            }
            else
            {
                *p_steer_angle = -m_model.max_steering_angle;
            }
            PRINT_INFO("[{:d}/{:d}] {:.3f}\n\n\n\n\n", m_cur_wait_index, total_wait_index, *p_steer_angle * 180 * M_1_PI);
        }
        else
        {
            m_pid_controller.reset();
            m_last_move_direction = m_direction_list[m_target_index];
            m_cur_wait_index = 0;
        }
    }
    else
    {
        m_cur_wait_index = 0;
    }
    // != 0 means in_change_move_direction_behavior
    return  m_cur_wait_index != 0;
}

}
