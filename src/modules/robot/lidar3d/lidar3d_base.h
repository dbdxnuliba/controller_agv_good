#pragma once


#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/data_types.h"
#include "robot/sensor_base.h"
#include "common/json.hpp"
#include "common/common.h"
#include "common/time_keeper.h"


namespace bz_robot
{

class Lidar3DBase : public SensorBase
{
    struct Lidar3DConfig
    {
        std::string name;
        VectorX3<FLOAT_T> pose_offset;
        VectorX3<FLOAT_T> angle_offset;
        std::vector<FLOAT_T> pose_x_range;
        std::vector<FLOAT_T> pose_y_range;
        std::vector<FLOAT_T> pose_z_range;
    };
public:
    bool import_config(const std::string &json_str)
    {
    	try
	    {

            nlohmann::json j = nlohmann::json::parse(json_str);;

            m_lidar3d_config.name = j["NAME"];
            m_lidar3d_config.pose_offset.x = j["POSE_OFFSET_X"];
            m_lidar3d_config.pose_offset.y = j["POSE_OFFSET_Y"];
            m_lidar3d_config.pose_offset.z = j["POSE_OFFSET_Z"];
            m_lidar3d_config.angle_offset.x = j["ANGLE_OFFSET_X"];
            m_lidar3d_config.angle_offset.y = j["ANGLE_OFFSET_Y"];
            m_lidar3d_config.angle_offset.z = j["ANGLE_OFFSET_Z"];
            m_lidar3d_config.angle_offset.x = degree_to_radian(m_lidar3d_config.angle_offset.x);
            m_lidar3d_config.angle_offset.y = degree_to_radian(m_lidar3d_config.angle_offset.y);
            m_lidar3d_config.angle_offset.z = degree_to_radian(m_lidar3d_config.angle_offset.z);
            std::vector<float> x_range_list = j["USED_X_POSE_RANGE"];
            std::vector<float> y_range_list = j["USED_Y_POSE_RANGE"];
            std::vector<float> z_range_list = j["USED_Z_POSE_RANGE"];
            m_lidar3d_config.pose_x_range = x_range_list;
            m_lidar3d_config.pose_y_range = y_range_list;
            m_lidar3d_config.pose_z_range = z_range_list;

            return true;
	    }
	    catch(std::exception& e )
	    {
	        PRINT_ERROR("exception: %s\n", e.what());
	    }
	    catch(...)
	    {
	        PRINT_ERROR("un expexted occured\n");
	    }
	    return false;
    }
    const std::string name() const {return m_lidar3d_config.name;}

    void set_data(const Msg<Laser3DData>& msg)
    {
        m_data.time_stamp_us = msg.time_stamp_us;
        m_data.data.points = std::move(laser3d_to_points(msg.data));
        m_data.data.msg_location = msg.data.msg_location;
    }

    std::vector<VectorX3<FLOAT_T>> laser3d_to_points(Laser3DData laser_data)
    {
        //RECORD_TIME();
        std::vector<VectorX3<FLOAT_T>> laser_points;
        Eigen::Vector3f euler(laser_data.angle_offset.z, laser_data.angle_offset.y, laser_data.angle_offset.x);

        Eigen::Matrix3f r;
        r = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitX());

        for(int i = 0; i < laser_data.points.size(); ++i)
        {
            FLOAT_T x = laser_data.points[i].x;
            FLOAT_T y = laser_data.points[i].y;
            FLOAT_T z = laser_data.points[i].z;
            if( (m_lidar3d_config.pose_x_range[0] < x && x < m_lidar3d_config.pose_x_range[1]) &&
                (m_lidar3d_config.pose_y_range[0] < y && y < m_lidar3d_config.pose_x_range[1]) &&
                (m_lidar3d_config.pose_z_range[0] < z && z < m_lidar3d_config.pose_z_range[1]) )
            {
                Eigen::Vector3f point(laser_data.points[i].x, laser_data.points[i].y, laser_data.points[i].z);
                point = r * point;
                FLOAT_T x = point[0] + laser_data.pose_offset.x;
                FLOAT_T y = point[1] + laser_data.pose_offset.y;
                FLOAT_T z = point[2] + laser_data.pose_offset.z;
                laser_points.emplace_back(VectorX3<FLOAT_T>(x, y, z));
            }

        }
        return laser_points;
    }
    const Msg<LaserData> data() const {return m_data;}

private:
    Lidar3DConfig m_lidar3d_config;
    Msg<Laser3DData> m_msg_lidar3d;
    Msg<LaserData> m_data;
};

}
