#pragma once

#include "common/data_types.h"
namespace bz_robot
{
class SensorManagerParams;
class SensorManager
{
public:
    SensorManager();
    ~SensorManager();
    bool import_config(const std::string &file_path);
    ReturnStatus update_laser_2d(const std::string& name);
    bool set_laser2d_data(const Msg<Laser2DData> &msg);
    bool laser2d_data(const std::string& name, Msg<LaserData> *p_msg);
    ReturnStatus update_laser_3d(const std::string& name);
    bool set_laser3d_data(const Msg<Laser3DData> &msg);
    bool laser3d_data(const std::string& name, Msg<LaserData> *p_msg);
public:
    SensorManagerParams *mp_params;
};

}
