#pragma once


#include <string>
#include "common/common.h"
#include "common/data_types.h"


namespace bz_robot
{

class TrackerBase
{
public:
    TrackerBase() {}
    virtual ~TrackerBase() {}
    virtual bool init() {return true;}
    virtual bool import_config(const char* file_path) {return true;}
    bool import_config(const std::string &file_path)    {return import_config(file_path.c_str());}
    virtual bool set_path(const Msg<PathData> &path, const Msg<ControlData> &msg_cur_control_data) {return true;}
    virtual bool set_goal(const Msg<Pose<FLOAT_T>>& goal) {return true;}
    virtual bool set_stop_signal(const Msg<bool>& stop) {return true;}
    virtual Msg<bool> is_goal_reached() = 0;
    virtual Msg<ControlData> scroll_calculate(const FLOAT_T &dt, const Msg<Pose<FLOAT_T>> &msg_location,
                                         const Msg<ControlData>& msg_cur_control_data) = 0;
};

}
