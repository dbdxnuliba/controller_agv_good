#pragma once

#include <stdio.h>
#include <string>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include "common/print.h"

#define PERIODIC_MS_TASK(X) PeriodicTask pt##__LINE__((X), __FILE__, __LINE__,std::string(__FUNCTION__));
#define SKIP_SLEEP()  pt##__LINE__.skip_sleep();
class PeriodicTask
{
public:
    PeriodicTask(int64_t time_ms, std::string file_name = __FILE__, int line_no = __LINE__, std::string function_name = __FUNCTION__)
    {
        this->m_time_us = time_ms * 1000;
        this->m_file_name = file_name;
        this->m_line_no = line_no;
        this->m_function_name = function_name;
        m_start_time = std::chrono::steady_clock::now();
    }

    ~PeriodicTask()
    {
        if(m_is_need_sleep)
        {
            auto end = std::chrono::steady_clock::now();
            int64_t tt = std::chrono::duration_cast<std::chrono::microseconds>(end - m_start_time).count();

            if(tt > m_time_us)
            {
                LOGGER.p_logger->warn("[{}]-[{}]:-[{}]: used {}ms, over than {}ms", this->m_file_name.c_str(), this->m_line_no, this->m_function_name.c_str(), tt * 0.001, m_time_us * 0.001);
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::microseconds(std::max((int64_t)0, (int64_t)(m_time_us - tt))));
            }
        }
    }

    void skip_sleep()
    {
        m_is_need_sleep = false;
    }

private:
    int64_t m_time_us;
    std::string m_file_name;
    int m_line_no;
    std::string m_function_name;
    std::chrono::steady_clock::time_point m_start_time;
    struct timespec m_start_ts;
    struct timespec m_stop_ts;
    clockid_t m_clock_id;
    bool m_is_need_sleep = true;
};

