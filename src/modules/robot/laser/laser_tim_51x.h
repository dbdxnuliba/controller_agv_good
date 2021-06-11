#pragma once

#include <mutex>
//#include "laser_data.h"
#include "common/data_types.h"
#include "robot/laser/laser_base.h"
namespace bz_robot
{

//class laser_tim_51x : public SensorBase<Laser2DData>
//{
//public:
//    laser_tim_51x();
//    Laser2DData data();
//};


class LaserTim51x : public LaserBase
{
public:
    LaserTim51x();
    void get_data(Laser2DData *data);
//private:
    bool import_config(const std::string &json_str);
    bool init(const char *uart_name);
    void run();

    ReturnStatus run_once();
    const Msg<Laser2DData> data() const;
    const std::string name() const;
private:
    std::recursive_mutex m_mtx;
    Laser2DData m_laser_data;
    std::string m_port_name;
    std::vector<std::vector<float> > m_angle_range_list;
    int m_speed;
    int m_flow_ctrl;
    int m_databits;
    int m_stopbits;
    int m_parity;
    bool m_is_need_request_msg;
    int m_uart_fd = -1;
    bool m_is_start = false;
    bool m_is_end = false;
    uint32_t m_start_index = 0;
    uint32_t m_end_index = 0;
    std::string m_part;
};
}
