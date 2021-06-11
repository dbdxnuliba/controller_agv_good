#pragma once

#include "common/data_types.h"
#include <memory>
namespace bz_robot
{

class MapBase;
class CollisionDetectorParams;
class CollisionDetector
{
public:
    enum class DriveMode
    {
        FREE_MODE = 0,
        BRAKE_MODE = 1,
        SLOWDOWN_MODE = 2
    };
public:
    CollisionDetector();
    bool import_config(const char* config_file);
    ControlData calc_safety_velocity(std::shared_ptr<bz_robot::MapBase> p_map, const ControlData &control_data, const Pose<FLOAT_T> &pose_robot);
    bool is_collision();
    bool enable_collision_detector();
private:
    DriveMode calc_drive_mode(std::shared_ptr<MapBase> p_map, const ControlData &control_data, const Pose<FLOAT_T> &pose_robot);
private:
    std::shared_ptr<CollisionDetectorParams> mp_params;
};

}
