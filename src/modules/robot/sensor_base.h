#pragma once

#include <mutex>
//#include "laser_data.h"
#include "common/data_types.h"

namespace bz_robot
{

//template <class T>
class SensorBase
{
public:
    SensorBase() {};
    virtual bool import_config(const std::string &json_str) {return true;}
    virtual bool init() {return true;}
    virtual ReturnStatus run_once() {return RET_SUCCESS;}
//    virtual void set_data(const T& data) {m_data = data;}
//    virtual const T data() const {return m_data;}
    virtual uint32_t update_cycle_time_ms() const {return 0;}
    virtual const std::string name() const {return m_name;}
private:
//    T m_data;
    std::string m_name;
};

}
