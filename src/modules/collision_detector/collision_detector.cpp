#include "collision_detector.h"
#include "common/data_types.h"
#include "map/map_base.h"
#include "modules/model/ackermann_model.h"

namespace bz_robot
{
namespace bz_robot {

}
class CollisionDetectorParams
{
public:
    CollisionDetectorParams()
    {
        brake_base_distance = 0.3;
        slowdown_distance = 2.0;
        is_collision = false;
        is_enable_collision_detector = true;
        prediction_time_s = 0.2;
    }
public:
    AckermannModel ackermann_model;
    FLOAT_T brake_base_distance = 0.3;
    FLOAT_T slowdown_distance = 2.0;
    bool is_collision;
    bool is_enable_collision_detector;
    FLOAT_T prediction_time_s = 0.2;
};

CollisionDetector::CollisionDetector()
{
    mp_params = std::make_shared<CollisionDetectorParams>();
    mp_params->is_enable_collision_detector = true;
}

bool CollisionDetector::import_config(const char *config_file)
{
    return true;
}

ControlData CollisionDetector::calc_safety_velocity(std::shared_ptr<MapBase> p_map, const ControlData &control_data, const Pose<FLOAT_T> &pose_robot)
{
    //ControlData &control_data = msg_control_data.data;
    //Pose<float> &pose_robot = msg_odom.data;
    if(!mp_params->is_enable_collision_detector)
    {
        return control_data;
    }
    ControlData safety_control_data = control_data;
    float velocity = 0;
    float steer_angle = 0;
    mp_params->is_collision = false;
    DriveMode drive_mode = DriveMode::FREE_MODE;
    //if(redis_get_control_data.get(&msg_control_data) && redis_get_odom.get(&msg_odom))
    {
        velocity = control_data.velocity;
        steer_angle = control_data.steer_angle;
        //AkermannModel akermann_model;
        drive_mode = calc_drive_mode(p_map, control_data, pose_robot);

        if(drive_mode == DriveMode::BRAKE_MODE)
        {
            mp_params->is_collision = true;
            safety_control_data.velocity = 0;
            safety_control_data.steer_angle = steer_angle;
        }
        else if(drive_mode == DriveMode::SLOWDOWN_MODE)
        {
            float new_velocity = 0;
            if(fabs(velocity) < 0.3)
            {
                new_velocity = velocity;
            }
            else
            {
                if(velocity > 0)
                {
                    new_velocity = 0.3;
                    //new_velocity = std::max(0.0, control_data[1] - fabs(4 * akermann_model.max_linear_acc * control_data[0]));
                }
                else if(velocity < 0)
                {
                    new_velocity = -0.3;
                    //new_velocity = std::min(0.0, control_data[1] + fabs(4 * akermann_model.max_linear_acc * control_data[0]));
                }
            }
            safety_control_data.velocity = new_velocity;
        }
        else
        {
            //control_data.velocity = 0;
        }

    }
    return safety_control_data;
}

bool CollisionDetector::is_collision()
{
    return mp_params->is_collision;
}

bool CollisionDetector::enable_collision_detector()
{
    return mp_params->is_enable_collision_detector;
}

CollisionDetector::DriveMode CollisionDetector::calc_drive_mode(std::shared_ptr<MapBase> p_map, const ControlData &control_data, const Pose<FLOAT_T> &pose_robot)
{
    float velocity = 0;
    float steer_angle = 0;
    //bool is_collision = false;
    DriveMode drive_mode = DriveMode::FREE_MODE;

    velocity = control_data.velocity;
    steer_angle = control_data.steer_angle;

    if(steer_angle == 0)
    {
        steer_angle = 0.0001;
        //懒得计算直线运动时候的情形了，后续再加上
    }
    if(steer_angle != 0)
    {
        float radius = mp_params->ackermann_model.wheel_base / fabs(tan(steer_angle));
        //机器人
        VectorX2<float> r_vector(radius * cos(pose_robot.heading_angle),radius * sin(pose_robot.heading_angle));
        //该向量为机器人中心与最小转弯半径对应圆心构成的
        VectorX2<float> offset_vector = rotate_2d(r_vector, steer_angle > 0? M_PI_2: -M_PI_2);
        //以最小半径旋转时的旋转中心
        VectorX2<float> circle_center = offset_vector + pose_robot.position;
        VectorX2<float> current_point;
        //计算机器人在栅格地图中的坐标
        p_map->world_to_map(pose_robot.position.x, pose_robot.position.y, &current_point.x, &current_point.y);
        VectorX2<float> map_circle_center;
        //计算旋转中心对应的栅格地图坐标
        p_map->world_to_map(circle_center.x, circle_center.y, &map_circle_center.x, &map_circle_center.y);
        //刹车距离mp_params->brake_base_distance为0.3m，附加部分由于dt未赋值，所以为0
        float brake_distance = mp_params->brake_base_distance + fabs(velocity) * mp_params->prediction_time_s;
        //std::cout<<"control_data.dt="<<control_data.dt<<std::endl;
        //更新真正的刹车距离
        float brake_distance_inflation_ratio = 1 + fabs(0.3 * velocity / (mp_params->ackermann_model.max_linear_velocity));
        if(velocity >= 0)
        {
            brake_distance = std::max(mp_params->ackermann_model.center_to_front * brake_distance_inflation_ratio, brake_distance);
        }
        else
        {
            brake_distance = std::max(mp_params->ackermann_model.center_to_back * brake_distance_inflation_ratio, brake_distance);
        }
        //刹车距离转化到地图坐标系
        brake_distance = brake_distance / p_map->resolution();
        float slowdown_distance = mp_params->slowdown_distance;
        slowdown_distance = slowdown_distance / p_map->resolution();
        radius = radius / p_map->resolution();
        //float brake_angle = fabs(brake_distance / radius);
        //float slowdown_angle = brake_angle + fabs(slowdown_distance / radius);
        //
        float brake_angle = fabs(atan2(brake_distance ,radius));
        //std::cout<<"分辨率："<<p_map->resolution()<<std::endl;
        //std::cout<<"刹车距离："<<brake_distance<<std::endl;
        //std::cout<<"转弯半径1："<<radius<<std::endl;
        //std::cout<<"刹车角度："<<brake_angle<<std::endl;
        float slowdown_angle = fabs(atan2(brake_distance+slowdown_distance ,radius));
        //std::cout<<"减速角度："<<slowdown_angle<<std::endl;
        const uint32_t map_w = p_map->size_in_cells_x();
        const uint32_t map_h = p_map->size_in_cells_y();
        //机器人中心到最左侧和最右侧的距离
        float left = mp_params->ackermann_model.center_to_left;
        float right = mp_params->ackermann_model.center_to_right;
        if(steer_angle > 0)
        {
            left = mp_params->ackermann_model.center_to_right;
            right = mp_params->ackermann_model.center_to_left;
        }
        left = left / p_map->resolution();
        right = right / p_map->resolution();
        uint8_t obstacles_value = p_map->obstacles_cost() - 1;
        //机器人中心与圆心连线的角度
        float angle_current = atan2(current_point.y - map_circle_center.y,
                                    current_point.x - map_circle_center.x);
        //进行局部地图的逐个搜索，找到距离最近的障碍物，进而判断行走的模式
        for(uint32_t x = 0; x < map_w; ++x)
        {
            if(drive_mode == DriveMode::BRAKE_MODE)
            {
                break;
            }
            for(uint32_t y = 0; y < map_h; ++y)
            {
              //检测到障碍物
              if(p_map->cost(x, y) > obstacles_value)
                {
                    if(hypot(x - current_point.x, y - current_point.y) < left)
                    {
                        PRINT_ERROR("obstacles in robot");
                    }

                    float distance = hypot(x - map_circle_center.x, y - map_circle_center.y);
                    //distance > radius - left && distance < radius + right？？？是否应该是+left，-right
                    if(distance > radius - left && distance < radius + right)
                    {
                        float angle_obstacles = atan2(y - map_circle_center.y, x - map_circle_center.x);
                        float angle_diff = angle_obstacles - angle_current;

                        angle_diff = constraint_angle_r(angle_diff, -M_PI, M_PI);
                        float sign = 1.0;
                        //左转 前进
                        if(velocity > 0 && steer_angle > 0)
                        {
                            sign = 1.0;
                        }
                        //右转 前进
                        else if(velocity > 0 && steer_angle < 0)
                        {
                            sign = -1.0;
                        }
                        //左转 后退
                        else if(velocity < 0 && steer_angle > 0)
                        {
                            sign = -1.0;
                        }
                        //右转 后退
                        else if(velocity < 0 && steer_angle < 0)
                        {
                            sign = 1.0;
                        }
                        else
                        {
                            //do nothing
                        }
                        angle_diff  = angle_diff * sign;
                        if(angle_diff < brake_angle && angle_diff >= 0)
                        {
                            PRINT_INFO("obstacles to robot = {}", hypot(x - current_point.x, y - current_point.y));
                            //brake
                            drive_mode = DriveMode::BRAKE_MODE;
                            PRINT_WARN("drive_mode = BRAKE_MODE");
                            break;
                        }
                        else if (angle_diff < slowdown_angle && angle_diff >= 0)
                        {
                            //slow down
                            drive_mode = DriveMode::SLOWDOWN_MODE;
                            //can't brake, may be exist obstacles in brake area
                            //break;
                        }
                        else
                        {
                            //no obstacles;
                        }
                    }
                }
            }
        }
    }
    return drive_mode;
}

#if 0
static bool collision_check(std::shared_ptr<MapBase> p_map, const Pose<float> &pose_robot)
{
    bool is_collision = false;
    FootPrintData footprint = p_map->robot_footprint();
    float min_x = DBL_MAX;
    float max_x = 0;
    float min_y = DBL_MAX;
    float max_y = 0;

    float cos_heading_angle = cos(pose_robot.heading_angle);
    float sin_heading_angle = sin(pose_robot.heading_angle);
    float map_pose_x = 0;
    float map_pose_y = 0;
    p_map->world_to_map(pose_robot.position.x, pose_robot.position.y, &map_pose_x, &map_pose_y);
    for(int i = 0; i < footprint.size(); i++)
    {
        VectorX2<float> new_pt;
        new_pt.x = (float)map_pose_x + (footprint[i].x * cos_heading_angle - footprint[i].y * sin_heading_angle)/p_map->resolution();
        new_pt.y = (float)map_pose_y + (footprint[i].x * sin_heading_angle + footprint[i].y * cos_heading_angle)/p_map->resolution();
        footprint[i] = new_pt;
        min_x = std::min(min_x, footprint[i].x);
        max_x = std::max(max_x, footprint[i].x);
        min_y = std::min(min_y, footprint[i].y);
        max_y = std::max(max_y, footprint[i].y);
    }

    uint32_t range_min_x = uint32_t(std::max(min_x, 0.0f));
    uint32_t range_max_x = uint32_t(std::min(uint32_t(max_x), p_map->size_in_cells_x() - 1));
    uint32_t range_min_y = uint32_t(std::max(min_y, 0.0f));
    uint32_t range_max_y = uint32_t(std::min(uint32_t(max_y), p_map->size_in_cells_y() - 1));

    uint8_t obstacle_cost = p_map->obstacles_cost();
    for(uint32_t x_pos = range_min_x; x_pos <= range_max_x; ++x_pos)
    {
        for(uint32_t y_pos = range_min_y; y_pos <= range_max_y; ++y_pos)
        {
            //if(x_pos < p_map->size_in_cells_x() && y_pos < p_map->size_in_cells_y())
            {
                if(p_map->cost(x_pos, y_pos) > obstacle_cost)
                {
                    VectorX2<float> world_pos;
                    p_map->map_to_world(x_pos, y_pos, &world_pos.x, &world_pos.y);
                    VectorX2<float> point((float(x_pos)), float (y_pos));
                    if(p_map->is_inside_robot_footprint(point, footprint))
                    {
                        is_collision = true;
                        PRINT_WARN("world pos({}, {}) DETECT COLLISION, cost = {}", world_pos.x, world_pos.y, p_map->cost(x_pos, y_pos));
                        break;
                    }
                }
            }
            if(is_collision)
            {
                break;
            }
        }
        if(is_collision)
        {
            break;
        }
    }
    return is_collision;
}

static DriveMode calc_drive_mode(const AkermannModel &akermann_model, const ControlData &control_data, const Pose<float> &pose_robot)
{
    float velocity = 0;
    float steer_angle = 0;
    bool is_collision = false;


    //Pose<float> pose_robot;
    DriveMode drive_mode = DriveMode::FREE_MODE;
//    if(!get_robot_pose(&pose_robot))
//    {
//        is_collision = true;
//        drive_mode = DriveMode::BRAKE_MODE;
//    }
//    else
    {
        LOCAL_MAP_MTX.lock();
        std::shared_ptr<MapBase> p_map = std::make_shared<GridMapInterface>(GRID_LOCAL_MAP_DATA);
        LOCAL_MAP_MTX.unlock();

        velocity = control_data.velocity;
        steer_angle = control_data.steer_angle;

        if(steer_angle == 0)
        {
            steer_angle = 0.0001;
            //懒得计算直线运动时候的情形了，后续再加上
        }
        if(steer_angle != 0)
        {
            float radius = akermann_model.wheel_base / fabs(tan(steer_angle));
            VectorX2<float> r_vector(radius * cos(pose_robot.heading_angle),
                                     radius * sin(pose_robot.heading_angle));
            VectorX2<float> offset_vector = rotate_2d(r_vector, steer_angle > 0? M_PI_2: -M_PI_2);
            VectorX2<float> circle_center = offset_vector + pose_robot.position;
            VectorX2<float> current_point;
            p_map->world_to_map(pose_robot.position.x, pose_robot.position.y, &current_point.x, &current_point.y);
            VectorX2<float> map_circle_center;
            p_map->world_to_map(circle_center.x, circle_center.y, &map_circle_center.x, &map_circle_center.y);
            //float brake_distance = 1.0;
            float brake_distance = 0.3 + fabs(velocity) * 2 * control_data.dt;
            float brake_distance_inflation_ratio = 1 + fabs(0.3 * velocity / (akermann_model.max_linear_velocity));
            if(velocity >= 0)
            {
                brake_distance = std::max(akermann_model.center_to_front * brake_distance_inflation_ratio, brake_distance);
            }
            else
            {
                brake_distance = std::max(akermann_model.center_to_back * brake_distance_inflation_ratio, brake_distance);
            }
            brake_distance = brake_distance / p_map->resolution();
            float slowdown_distance = 3.0;
            slowdown_distance = slowdown_distance / p_map->resolution();
            radius = radius / p_map->resolution();
            //float brake_angle = fabs(brake_distance / radius);
            //float slowdown_angle = brake_angle + fabs(slowdown_distance / radius);
            float brake_angle = fabs(atan2(brake_distance ,radius));
            float slowdown_angle = fabs(atan2(brake_distance+slowdown_distance ,radius));
            const uint32_t map_w = p_map->size_in_cells_x();
            const uint32_t map_h = p_map->size_in_cells_y();

            float left = akermann_model.center_to_left;
            float right = akermann_model.center_to_right;
            if(steer_angle > 0)
            {
                left = akermann_model.center_to_right;
                right = akermann_model.center_to_left;
            }
            left = left / p_map->resolution();
            right = right / p_map->resolution();
            uint8_t obstacles_value = p_map->obstacles_cost() - 1;
            float angle_current = atan2(current_point.y - map_circle_center.y,
                                        current_point.x - map_circle_center.x);
            for(uint32_t x = 0; x < map_w; ++x)
            {
                if(drive_mode == DriveMode::BRAKE_MODE)
                {
                    break;
                }
                for(uint32_t y = 0; y < map_h; ++y)
                {
                    if(p_map->cost(x, y) > obstacles_value)
                    {
                        float distance = hypot(x - map_circle_center.x, y - map_circle_center.y);
                        if(distance > radius - left && distance < radius + right)
                        {
                            float angle_obstacles = atan2(y - map_circle_center.y, x - map_circle_center.x);
                            float angle_diff = angle_obstacles - angle_current;

                            angle_diff = constraint_angle_r(angle_diff, -M_PI, M_PI);
                            float sign = 1.0;
                            //左转 前进
                            if(velocity > 0 && steer_angle > 0)
                            {
                                sign = 1.0;
                            }
                            //右转 前进
                            else if(velocity > 0 && steer_angle < 0)
                            {
                                sign = -1.0;
                            }
                            //左转 后退
                            else if(velocity < 0 && steer_angle > 0)
                            {
                                sign = -1.0;
                            }
                            //右转 后退
                            else if(velocity < 0 && steer_angle < 0)
                            {
                                sign = 1.0;
                            }
                            else
                            {
                                //do nothing
                            }
                            angle_diff  = angle_diff * sign;
                            if(angle_diff < brake_angle && angle_diff >= 0)
                            {
                                //brake
                                drive_mode = DriveMode::BRAKE_MODE;
                                PRINT_WARN("drive_mode = BRAKE_MODE");
                                break;
                            }
                            else if (angle_diff < slowdown_angle && angle_diff >= 0)
                            {
                                //slow down
                                drive_mode = DriveMode::SLOWDOWN_MODE;
                                //can't brake, may be exist obstacles in brake area
                                //break;
                            }
                            else
                            {
                                //no obstacles;
                            }
                        }
                    }
                }
            }
        }
    }
    return drive_mode;
}

static Msg<ControlData> calc_safety_velocity(Msg<ControlData> msg_control_data,  Msg<Pose<float>> msg_odom)
{
    ControlData &control_data = msg_control_data.data;
    Pose<float> &pose_robot = msg_odom.data;
    float velocity = 0;
    float steer_angle = 0;
    bool is_collision = false;
    DriveMode drive_mode = DriveMode::BRAKE_MODE;
    if(redis_get_control_data.get(&msg_control_data) && redis_get_odom.get(&msg_odom))
    {
        velocity = msg_control_data.data.velocity;
        steer_angle = msg_control_data.data.steer_angle;
        AkermannModel akermann_model;
        DriveMode drive_mode = calc_drive_mode(akermann_model, control_data, pose_robot);

        if(drive_mode == DriveMode::BRAKE_MODE)
        {
            is_collision = true;
            control_data.velocity = 0;
            control_data.steer_angle = steer_angle;
        }
        else if(drive_mode == DriveMode::SLOWDOWN_MODE)
        {
            float new_velocity = 0;
            if(fabs(velocity) < 0.3)
            {
                new_velocity = velocity;
            }
            else
            {
                if(velocity > 0)
                {
                    new_velocity = 0.3;
                    //new_velocity = std::max(0.0, control_data[1] - fabs(4 * akermann_model.max_linear_acc * control_data[0]));
                }
                else if(velocity < 0)
                {
                    new_velocity = -0.3;
                    //new_velocity = std::min(0.0, control_data[1] + fabs(4 * akermann_model.max_linear_acc * control_data[0]));
                }
            }
            control_data.velocity = new_velocity;
        }
        else
        {
            //control_data.velocity = 0;
        }

        IS_COLLISION = is_collision;
        if(IS_STOP_ROBOT)
        {
            control_data.velocity = 0;
            control_data.steer_angle = steer_angle;
        }
    }
    return control_data;
}
#endif

}
