#include <fstream>
#include "tracker/pid/tracker_pid.h"
#include "tracker/pid/pid.h"
#include "common/json.hpp"
#include "model/ackermann_model.h"

namespace bz_robot
{

class TrackerParams
{
public:
    AckermannModel model;
    std::shared_ptr<PID> p_pid;
    Msg<PathData> path;
    Msg<Pose<FLOAT_T>> goal;
    FLOAT_T pid_p;
    FLOAT_T pid_i;
    FLOAT_T pid_d;

};

TrackerPid::TrackerPid()
{
    mp_params = std::make_shared<TrackerParams>();
    mp_params->p_pid = std::make_shared<PID>();
}

bool TrackerPid::import_config(const char *file_path)
{
    try
    {
        std::ifstream i(file_path);

        if(i)
        {
            nlohmann::json j;
            i >> j;
            std::string config_ackermann_model = j["AKERMANN_CONFIG_FILE_PATH"];
            mp_params->pid_p = j["P"];
            mp_params->pid_i = j["I"];
            mp_params->pid_d = j["D"];
            mp_params->model.read_params_from_file(config_ackermann_model);
            mp_params->p_pid->set_model(mp_params->model);
            mp_params->p_pid->set_pid(mp_params->pid_p, mp_params->pid_i, mp_params->pid_d);
            return true;
        }
        else
        {
            printf("can't read config files from: %s\n", file_path);
        }
    }
    catch( std::exception& e )
    {
        printf("%s\n", e.what());
    }
    catch(...)
    {
        printf("un expexted occured\n");
    }

    return false;
}

bool TrackerPid::set_path(const Msg<PathData> &path, const Msg<ControlData> &msg_cur_control_data)
{
    mp_params->path = path;
    mp_params->p_pid->set_path(path.data, msg_cur_control_data.data);
    return true;
}

bool TrackerPid::set_goal(const Msg<Pose<FLOAT_T>> &goal)
{
    mp_params->goal = goal;
    return true;
}

Msg<bool> TrackerPid::is_goal_reached()
{
  Msg<bool> msg;
  msg.data = true;
  return msg;
}

bool TrackerPid::set_stop_signal(const Msg<bool> &stop)
{
  return true;
}

Msg<ControlData> TrackerPid::scroll_calculate(const FLOAT_T &dt, const Msg<Pose<FLOAT_T>> &msg_location, const Msg<ControlData> &msg_cur_control_data)
{
    Msg<ControlData> msg;
    msg.time_stamp_us = time_stamp_us();
    msg.data.dt = dt;
    ControlData control_param = mp_params->p_pid->run_once(msg_location.data, msg_cur_control_data.data.velocity, msg_cur_control_data.data.steer_angle, dt);
    msg.data = std::move(control_param);
    return msg;
}


}
