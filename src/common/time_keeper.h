#ifndef __TIME_KEEPER__
#define __TIME_KEEPER__

#include <stdio.h>
#include <string>
#include <sys/time.h>
#include "common/print.h"

namespace bz_robot
{
#define __RECORD_TIME(A, X) TimeKeeper kp##A(__FILE__, __LINE__,std::string(__FUNCTION__)+std::string(" ")+std::string(X));
#define __RECORD_TIME2(A, X) __RECORD_TIME(A, X)

#define RECORD_TIME(X) __RECORD_TIME2(__LINE__, X)


class TimeKeeper
{
public:
    TimeKeeper(std::string file_name = __FILE__, int line_no = __LINE__, std::string function_name = __FUNCTION__)
    {
        this->m_file_name = file_name;
        this->m_line_no = line_no;
        this->m_function_name = function_name;
        this->m_clock_id = CLOCK_REALTIME;
        this->m_clock_id = CLOCK_PROCESS_CPUTIME_ID;
        this->m_clock_id = CLOCK_MONOTONIC;
        clock_gettime(m_clock_id, &(this->m_start_ts));
    }

    ~TimeKeeper()
    {
        clock_gettime(m_clock_id, &(this->m_stop_ts));
        double used_msec = 1000.0 * (m_stop_ts.tv_sec - m_start_ts.tv_sec) + (m_stop_ts.tv_nsec - m_start_ts.tv_nsec) / (1.0 * 1000 * 1000);
        LOGGER.p_logger->info("[{}]-[{}]:-[{}]: used {:.1f}ms \n", this->m_file_name.c_str(), this->m_line_no, this->m_function_name.c_str(), used_msec);
    }

    const double calc_time_it() const
    {
        struct timespec ts;
        clock_gettime(m_clock_id, &ts);
        double used_sec = (ts.tv_sec - m_start_ts.tv_sec) + (ts.tv_nsec - m_start_ts.tv_nsec) / (1.0 * 1000 * 1000 * 1000);
        return used_sec;
    }
private:
    std::string m_file_name;
    int m_line_no;
    std::string m_function_name;

    struct timespec m_start_ts;
    struct timespec m_stop_ts;
    clockid_t m_clock_id;

};

}
#endif
