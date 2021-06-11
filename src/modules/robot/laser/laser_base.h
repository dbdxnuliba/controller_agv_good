#pragma once

#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/data_types.h"
#include "robot/sensor_base.h"
#include "common/common.h"
#include "common/json.hpp"
#include "common/print.h"


namespace bz_robot
{

class LaserBase : public SensorBase
{
    struct Laser2DConfig
    {
        std::string name;
        VectorX3<FLOAT_T> pose_offset;
        VectorX3<FLOAT_T> angle_offset;
        FLOAT_T start_angle = 0;
        FLOAT_T angle_step = 0;
        std::vector<std::vector<FLOAT_T>> valid_angle_angle_list;
    };
public:
    bool import_config(const std::string &json_str)
    {
    	try
	    {

            nlohmann::json j = nlohmann::json::parse(json_str);;

            m_laser2d_config.name = j["NAME"];
            m_laser2d_config.pose_offset.x = j["POSE_OFFSET_X"];
            m_laser2d_config.pose_offset.y = j["POSE_OFFSET_Y"];
            m_laser2d_config.pose_offset.z = j["POSE_OFFSET_Z"];
            m_laser2d_config.angle_offset.x = j["ANGLE_OFFSET_X"];
            m_laser2d_config.angle_offset.y = j["ANGLE_OFFSET_Y"];
            m_laser2d_config.angle_offset.z = j["ANGLE_OFFSET_Z"];
            m_laser2d_config.angle_offset.x = degree_to_radian(m_laser2d_config.angle_offset.x);
            m_laser2d_config.angle_offset.y = degree_to_radian(m_laser2d_config.angle_offset.y);
            m_laser2d_config.angle_offset.z = degree_to_radian(m_laser2d_config.angle_offset.z);
            std::vector<std::vector<float> > angle_range_list = j["USED_ANGLE_RANGE"];
            for(uint32_t i = 0; i < angle_range_list.size(); ++i)
            {
                angle_range_list[i][0] = degree_to_radian(angle_range_list[i][0]);
                angle_range_list[i][1] = degree_to_radian(angle_range_list[i][1]);
            }
            m_laser2d_config.valid_angle_angle_list = angle_range_list;


            m_laser_data.data.name = m_laser2d_config.name;
            return true;
	    }
	    catch(std::exception& e )
	    {
            PRINT_ERROR("{}", json_str);
            PRINT_ERROR("exception: {}", e.what());
	    }
	    catch(...)
	    {
            PRINT_ERROR("{}", json_str);
            PRINT_ERROR("un expexted occured");
	    }
	    return false;
    }

    void set_data(const Msg<Laser2DData>& msg)
    {
        m_laser2d_data.time_stamp_us = msg.time_stamp_us;
        m_laser2d_data.data.distance_list = msg.data.distance_list;
        m_laser2d_data.data.msg_location = msg.data.msg_location;
        m_laser2d_data.data.start_angle = msg.data.start_angle;
        m_laser2d_data.data.angle_step = msg.data.angle_step;

        m_laser_data.time_stamp_us = msg.time_stamp_us;
        m_laser_data.data.points = std::move(laser2d_to_point_cloud(msg.data));
        m_laser_data.data.msg_location = msg.data.msg_location;
    }

    void filtrate_laser2d_angles(Laser2DData* p_laser_data)
    {
        int angle_index = 0;
        for(int i = 0; i < p_laser_data->distance_list.size(); ++i)
        {
            float angle = p_laser_data->start_angle + i * p_laser_data->angle_step;
            if(angle < m_laser2d_config.valid_angle_angle_list[angle_index][0])
            {
                p_laser_data->distance_list[i] = 0;
            }
            else if(angle > m_laser2d_config.valid_angle_angle_list[angle_index][1])
            {
                if(angle_index + 1 < m_laser2d_config.valid_angle_angle_list.size())
                {
                    if(angle < m_laser2d_config.valid_angle_angle_list[angle_index+1][0])
                    {
                        p_laser_data->distance_list[i] = 0;
                    }
                    else
                    {
                        ++angle_index;
                    }
                }
                else
                {
                    p_laser_data->distance_list[i] = 0;
                }
            }
            else
            {
                //do nothing;
            }
        }
    }

    std::vector<VectorX3<FLOAT_T>> laser2d_to_point_cloud(Laser2DData laser_data)
    {
        std::vector<VectorX3<FLOAT_T>> laser_point_cloud;

        filtrate_laser2d_angles(&laser_data);

        Eigen::Vector3f euler(m_laser2d_config.angle_offset.z, m_laser2d_config.angle_offset.y, m_laser2d_config.angle_offset.x);

        Eigen::Matrix3f r;
        r = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitX());

        for(int i = 0; i < laser_data.distance_list.size(); ++i)
        {
            const FLOAT_T distance = laser_data.distance_list[i];

            if(distance != 0)
            {
                FLOAT_T x = distance * cos(laser_data.start_angle + i * laser_data.angle_step);
                FLOAT_T y = distance * sin(laser_data.start_angle + i * laser_data.angle_step);
                //PRINT_DEBUG("ori: {}, {}", x, y);
                FLOAT_T z = 0;
                Eigen::Vector3f point(x, y, z);
                point = r * point;
                x = point[0] + m_laser2d_config.pose_offset.x;
                y = point[1] + m_laser2d_config.pose_offset.y;
                z = point[2] + m_laser2d_config.pose_offset.z;
                laser_point_cloud.emplace_back(VectorX3<FLOAT_T>(x, y, z));
            }
        }
        return laser_point_cloud;
    }
    const Msg<LaserData> data() const {return m_laser_data;}
    const std::string name() const {return m_name;}
private:
	std::string m_name;
    Msg<Laser2DData> m_laser2d_data;
    Laser2DConfig m_laser2d_config;
    Msg<LaserData> m_laser_data;
    ReturnStatus m_status;
    //std::vector<std::vector<float>> m_angle_range_list;
};

}
