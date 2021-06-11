#include "modules/robot/laser/laser_tim_51x.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "common/common.h"
#include "common/json.hpp"
#include "common/print.h"
#include "common/uart.h"

namespace bz_robot
{

uint32_t str_hex_to_uint32(const std::string &hex)
{
    uint32_t x = 0;
    std::stringstream ss;
    ss << std::hex << hex;
    ss >> x;
    return x;
}

unsigned long splitPacket(const std::string &txt, std::vector<std::string> &strs, char ch)
{
    unsigned long pos = txt.find(ch);
    unsigned long initialPos = 0;
    strs.clear();

    // Decompose statement
    while (pos != std::string::npos)
    {
        strs.push_back(txt.substr(initialPos, pos - initialPos));
        initialPos = pos + 1;

        pos = txt.find(ch, initialPos);
    }

    // Add the last one
    strs.push_back(txt.substr(initialPos, std::min(pos, txt.size()) - initialPos + 1));
    return strs.size();
}
//inline float radian(float degree)
//{
//    return degree * M_PI / 180.0;
//}
//inline float degree(float radian)
//{
//    return radian * 180 / M_PI;
//}

void decode(const std::string &packet, Laser2DData *p_data)
{
    std::vector<std::string> splitted;
    unsigned long len = splitPacket(packet, splitted, ' ');
    //	LOG_DEBUG("str length {:d}", len);
    if (len < 27)
    {
        PRINT_ERROR("str length %lu < 27, to skip", len);
        return;
    }
    //	STX = splitted.at(0);
    //	LMD = splitted.at(1);
    //	device_version = splitted.at(2);
    //	device_ID = splitted.at(3);
    //	device_serial_0 = splitted.at(4);
    //	device_serial_1 = splitted.at(5);
    //	device_status = splitted.at(6);
    //	scan_angle = splitted.at(7);
    //	scan_count = splitted.at(8);
    //	scan_time = splitted.at(9);
    //	switch_input_status_0 = splitted.at(10);
    //	siwtch_input_status_1 = splitted.at(11);
    //	switch_output_status_0 = splitted.at(12);
    //	switch_output_status_1 = splitted.at(13);
    //	checksum_0 = splitted.at(14);
    //	checksum_1 = splitted.at(15);
    //	checksum_2 = splitted.at(16);
    //	scan_rate = splitted.at(17);
    //	encoder_input_status_0 = splitted.at(18);
    //	encoder_input_status_1 = splitted.at(19);
    //	output_channel = splitted.at(20);
    //	first_level_echo_measurement = splitted.at(21);
    //	start_angle_0 = splitted.at(22);
    //	start_angle_1 = splitted.at(23);
    //	start_angle_2 = splitted.at(24);
    //	angle_resolution = splitted.at(25);
    int start_angle = str_hex_to_uint32(splitted.at(24));
    int angle_step = str_hex_to_uint32(splitted.at(25));
    int measured_data_count = str_hex_to_uint32(splitted.at(26));
    //PRINT_DEBUG("measured_data_count = {:d}", measured_data_count);
    p_data->start_angle = degree_to_radian(start_angle * 0.0001);
    p_data->angle_step = degree_to_radian(angle_step * 0.0001);
    p_data->distance_list.assign(measured_data_count, 0);
    for (int i = 0; i < measured_data_count && (27 + i) < splitted.size(); i++)
    {
        p_data->distance_list[i] = str_hex_to_uint32(splitted.at(27 + i)) * 0.001;
        //printf("%2.2f\t", p_data->distance_list[i]);
    }
    //printf("\n");
}


LaserTim51x::LaserTim51x():
    m_is_need_request_msg(true)
{
//    if(import_config(config_file))
//    {
//        std::thread *thread_recv_data = new std::thread(&LaserTim51x::run, this);
//        thread_recv_data->detach();
//    }
}

void LaserTim51x::get_data(Laser2DData *data)
{
    std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
    *data = m_laser_data;
    m_laser_data.distance_list.clear();
}

bool LaserTim51x::import_config(const std::string &json_str)
{
    try
    {
//        PRINT_INFO("%s", file);
//        std::ifstream i(file);
//        if(i)
//        {
            nlohmann::json j = nlohmann::json::parse(json_str);;
//            j.parse(file);
//            i >> j;
            m_port_name = j["DRIVER_INFO"]["PORT_NAME"];
            m_laser_data.name = j["NAME"];
            m_laser_data.pose_offset.x = j["POSE_OFFSET_X"];
            m_laser_data.pose_offset.y = j["POSE_OFFSET_Y"];
            m_laser_data.pose_offset.z = j["POSE_OFFSET_Z"];
            m_laser_data.angle_offset.x = j["ANGLE_OFFSET_X"];
            m_laser_data.angle_offset.y = j["ANGLE_OFFSET_Y"];
            m_laser_data.angle_offset.z = j["ANGLE_OFFSET_Z"];
            m_laser_data.angle_offset.x = degree_to_radian(m_laser_data.angle_offset.x);
            m_laser_data.angle_offset.y = degree_to_radian(m_laser_data.angle_offset.y);
            m_laser_data.angle_offset.z = degree_to_radian(m_laser_data.angle_offset.z);
            //m_laser_data.is_mirror = j["IS_MIRROR"];
            //get used_angle_range
            std::vector<std::vector<float> > angle_range_list = j["USED_ANGLE_RANGE"];
            for(uint32_t i = 0; i < angle_range_list.size(); ++i)
            {
                angle_range_list[i][0] = degree_to_radian(angle_range_list[i][0]);
                angle_range_list[i][1] = degree_to_radian(angle_range_list[i][1]);
            }
            m_angle_range_list = angle_range_list;
            return true;
//        }
//        else
//        {
//            PRINT_ERROR("can't read config files from: %s\n", file);
//        }
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

bool LaserTim51x::init(const char* uart_name)
{
    int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
    int err;               //返回调用函数的状态

    //char port_name[100] = "/dev/ttyUSB0";
    fd = uart_open(fd, uart_name); //打开串口，返回文件描述符
    // fd=open("dev/ttyS1", O_RDWR);
    //printf("fd= \n",fd);
    do
    {
        err = uart_init(fd,115200,0,8,1,'N');
        //printf("Set Port Exactly!\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }while(-1 == err || -1 == fd);
    if(-1 == err)
    {
        fd = -1;
    }
    m_uart_fd = fd;
    return true;
}

void LaserTim51x::run()
{
    int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
//    int err;               //返回调用函数的状态
    int len;
    int i;
    char rcv_buf[1000];
    init(m_port_name.c_str());
    fd = m_uart_fd;
    while (1) //循环读取数据
    {
        std::string cmd_read_single = "\x02sRN LMDscandata\x03";
        len = uart_send(fd, cmd_read_single.data(), cmd_read_single.size());
        if(len < 0)
        {
            continue;
        }

        bool is_start = false;
        bool is_end = false;
        uint32_t start_index = 0;
        uint32_t end_index = 0;
        std::string part;
        while (1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            len = uart_recv(fd, rcv_buf,1000);
            if(len > 0)
            {
                start_index = 0;
                end_index = len-1;
                std::string recv_str;
                recv_str.insert(0, rcv_buf, len);
                for (i = 0; i < len; ++i)
                {
                    if (!is_start)
                    {
                        if (rcv_buf[i] == 0x02)
                        {
                            is_start = true;
                            start_index = i;
                        }
                    }
                    else
                    {
                        if (rcv_buf[i] == 0x03)
                        {
                            is_end = true;
                            end_index = i;
                            break;
                        }
                    }
                }
                part.insert(part.size(), recv_str, start_index, end_index - start_index+1);
                if(is_end)
                {
                    break;
                }
            }
        }

        Laser2DData laser_data;
        decode(part, &laser_data);
        //filter data
        int angle_index = 0;

        for(int i = 0; i < laser_data.distance_list.size(); ++i)
        {
            float angle = laser_data.start_angle + i * laser_data.angle_step;
            if(angle < m_angle_range_list[angle_index][0])
            {
                laser_data.distance_list[i] = 0;
            }
            else if(angle > m_angle_range_list[angle_index][1])
            {
                if(angle_index + 1 < m_angle_range_list.size())
                {
                    if(angle < m_angle_range_list[angle_index+1][0])
                    {
                        laser_data.distance_list[i] = 0;
                    }
                    else
                    {
                        ++angle_index;
                    }
                }
                else
                {
                    laser_data.distance_list[i] = 0;
                }
            }
            else
            {
                //do nothing;
            }
        }


        std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
        m_laser_data.start_angle = laser_data.start_angle;
        m_laser_data.angle_step = laser_data.angle_step;
        m_laser_data.distance_list = laser_data.distance_list;
    }
    uart_close(fd);
}

ReturnStatus LaserTim51x::run_once()
{
    if(m_is_need_request_msg)
    {
        //while (1)
        {
            const std::string cmd_read_single = "\x02sRN LMDscandata\x03";
            const int len = uart_send(m_uart_fd, cmd_read_single.data(), cmd_read_single.size());
            if(len < 0)
            {
                m_is_need_request_msg = true;
            }
            else
            {
                m_is_need_request_msg = false;
                m_is_start = false;
                m_is_end = false;
                m_start_index = 0;
                m_end_index = 0;
                m_part.clear();
            }
        }
    }
    else
    {
        char rcv_buf[1000];
        const int len = uart_recv(m_uart_fd, rcv_buf,1000);
        if(len > 0)
        {
            m_start_index = 0;
            m_end_index = len-1;
            std::string recv_str;
            recv_str.insert(0, rcv_buf, len);
            for (int i = 0; i < len; ++i)
            {
                if (!m_is_start)
                {
                    if (rcv_buf[i] == 0x02)
                    {
                        m_is_start = true;
                        m_start_index = i;
                    }
                }
                else
                {
                    if (rcv_buf[i] == 0x03)
                    {
                        m_is_end = true;
                        m_end_index = i;
                        break;
                    }
                }
            }
            m_part.insert(m_part.size(), recv_str, m_start_index, m_end_index - m_start_index + 1);
            if(m_is_end)
            {
                Laser2DData laser_data;
                decode(m_part, &laser_data);
                //filter data
                int angle_index = 0;

                for(int i = 0; i < laser_data.distance_list.size(); ++i)
                {
                    float angle = laser_data.start_angle + i * laser_data.angle_step;
                    if(angle < m_angle_range_list[angle_index][0])
                    {
                        laser_data.distance_list[i] = 0;
                    }
                    else if(angle > m_angle_range_list[angle_index][1])
                    {
                        if(angle_index + 1 < m_angle_range_list.size())
                        {
                            if(angle < m_angle_range_list[angle_index+1][0])
                            {
                                laser_data.distance_list[i] = 0;
                            }
                            else
                            {
                                ++angle_index;
                            }
                        }
                        else
                        {
                            laser_data.distance_list[i] = 0;
                        }
                    }
                    else
                    {
                        //do nothing;
                    }
                }
                m_is_need_request_msg = true;
                std::lock_guard<std::recursive_mutex> temp_lock(m_mtx);
                m_laser_data.start_angle = laser_data.start_angle;
                m_laser_data.angle_step = laser_data.angle_step;
                m_laser_data.distance_list = laser_data.distance_list;
                return RET_SUCCESS;
            }
        }
    }
    return RET_IS_BUSY;
}

const Msg<Laser2DData> LaserTim51x::data() const
{
    Msg<Laser2DData> msg;
    msg.time_stamp_us = time_stamp_us();
    msg.data = m_laser_data;
    return msg;
}

const std::string LaserTim51x::name() const
{
    return m_laser_data.name;
}
}
