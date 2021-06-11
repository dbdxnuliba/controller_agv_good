#include "robot/sensor_manager.h"
#include <unordered_map>
#include <string>
#include "common/json.hpp"
#include "common/print.h"
#include "robot/sensor_base.h"
#include "robot/laser/laser_tim_51x.h"
#include "robot/laser/laser_base.h"
#include "robot/lidar3d/lidar3d_base.h"

namespace bz_robot
{

class SensorManagerParams
{
public:
    std::unordered_map<std::string, LaserBase*> laser2d_sensors;
    std::unordered_map<std::string, Lidar3DBase*> laser3d_sensors;
//    std::unordered_map<std::string, Laser2DParams> laser_2d_params;
//    std::unordered_map<std::string, Laser3DParams> laser_3d_params;
//    std::unordered_map<std::string, std::unordered_map<int64_t, std::vector<Pose<int64_t> > > > laser_data_map;
};


SensorManager::SensorManager()
{
    mp_params = new SensorManagerParams();
}

SensorManager::~SensorManager()
{
    delete mp_params;
}

bool SensorManager::import_config(const std::string &file_path)
{
    bool result = false;
    try
    {
        std::ifstream i(file_path);
        if(i)
        {
            nlohmann::json j;
            i >> j;
            const std::string &laser_2d_file = j["LASER_2D_LIST"];
            std::ifstream i_laser_2d(laser_2d_file);
            if(i_laser_2d)
            {
                nlohmann::json j_laser_2d;
                i_laser_2d >> j_laser_2d;
                std::vector<nlohmann::json> laser2d_config = j_laser_2d["LASER_2D"];
                for(int i = 0; i < laser2d_config.size(); ++i)
                {
                    nlohmann::json &j_laser2d_instance = laser2d_config[i];
                    if(j_laser2d_instance["TYPE"] == "TIM51X")
                    {
                        mp_params->laser2d_sensors[j_laser2d_instance["NAME"]] = new LaserTim51x();
                        mp_params->laser2d_sensors[j_laser2d_instance["NAME"]]->import_config(laser2d_config[i].dump());
                    }
                    else if(j_laser2d_instance["TYPE"] == "ROS")
                    {
                        mp_params->laser2d_sensors[j_laser2d_instance["NAME"]] = new LaserBase();
                        mp_params->laser2d_sensors[j_laser2d_instance["NAME"]]->import_config(laser2d_config[i].dump());
                    }
                }
            }
            else
            {
                PRINT_ERROR("can't read config files from: {}\n", laser_2d_file);
            }

            const std::string &laser_3d_file = j["LASER_3D_LIST"];
            std::ifstream i_laser_3d(laser_3d_file);
            if(i_laser_3d)
            {
                nlohmann::json j_laser_3d;
                i_laser_3d >> j_laser_3d;
                std::vector<nlohmann::json> laser3d_config = j_laser_3d["LASER_3D"];
                for(int i = 0; i < laser3d_config.size(); ++i)
                {
                    nlohmann::json &j_laser3d_instance = laser3d_config[i];
                    if(j_laser3d_instance["TYPE"] == "ROS")
                    {
                        mp_params->laser3d_sensors[j_laser3d_instance["NAME"]] = new Lidar3DBase();
                        mp_params->laser3d_sensors[j_laser3d_instance["NAME"]]->import_config(laser3d_config[i].dump());
                    }
                }
            }
            else
            {
                PRINT_ERROR("can't read config files from: {}\n", laser_3d_file);
            }

        }

    }
    catch(std::exception& e )
    {
        PRINT_ERROR("exception: {}\n", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("unexpexted occured\n");
    }
    return result;
}

ReturnStatus SensorManager::update_laser_2d(const std::string& name)
{
    ReturnStatus result = RET_ERROR;
    auto it = mp_params->laser2d_sensors.find(name);
    if(it != mp_params->laser2d_sensors.end())
    {
        result = ((it->second))->run_once();
    }
    return result;
}

bool SensorManager::set_laser2d_data(const Msg<Laser2DData> &msg)
{
    auto it = mp_params->laser2d_sensors.find(msg.data.name);
    if(it == mp_params->laser2d_sensors.end())
    {
        return false;
    }
    else
    {
        ((it->second))->set_data(msg);
        return true;
    }
    return false;
}

bool SensorManager::laser2d_data(const std::string &name, Msg<LaserData> *p_msg)
{
    bool result = false;
    auto it = mp_params->laser2d_sensors.find(name);
    if(it != mp_params->laser2d_sensors.end())
    {
        *p_msg = ((it->second))->data();
        result = true;
    }
    return result;
}

ReturnStatus SensorManager::update_laser_3d(const std::string &name)
{
    ReturnStatus result = RET_ERROR;
    auto it = mp_params->laser3d_sensors.find(name);
    if(it != mp_params->laser3d_sensors.end())
    {
        result = ((it->second))->run_once();
    }
    return result;
}

bool SensorManager::set_laser3d_data(const Msg<Laser3DData> &msg)
{
    auto it = mp_params->laser3d_sensors.find(msg.data.name);
    if(it == mp_params->laser3d_sensors.end())
    {
        return false;
    }
    else
    {
        ((it->second))->set_data(msg);
        return true;
    }
    return false;
}

bool SensorManager::laser3d_data(const std::string &name, Msg<LaserData> *p_msg)
{
    bool result = false;
    auto it = mp_params->laser3d_sensors.find(name);
    if(it != mp_params->laser3d_sensors.end())
    {
        *p_msg = ((it->second))->data();
        result = true;
    }
    return result;
}

}
