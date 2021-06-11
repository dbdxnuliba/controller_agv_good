#pragma once

#include <atomic>
#include <string>
//#include <event2/event.h>
#include "common/data_types.h"


namespace bz_robot
{
class TaskBase
{
public:
    TaskBase() {}
    virtual ~TaskBase() {}
    virtual bool import_config(const char* file_path){return true;}
    bool import_config(const std::string &file_path){return import_config(file_path.c_str());}
    virtual bool init() {return true;}
    virtual bool set_cmd(const std::string &cmd){return true;}
//    virtual bool run() {return true;}
    virtual RetMsg<std::string> run() {RetMsg<std::string> ret_msg; ret_msg.return_status = RET_SUCCESS; return ret_msg;}
    virtual bool clear() {return true;}
    virtual bool register_background_tasks() {return false;}
    virtual bool stop() {return true;}
};
}
