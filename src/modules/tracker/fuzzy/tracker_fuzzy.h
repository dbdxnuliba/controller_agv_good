#pragma once
#include "tracker/tracker_base.h"
#include "common/json.hpp"
#include "model/ackermann_model.h"
#include "fuzzy_controller.h"
namespace bz_robot
{
class TrackerParams;
class tracker_fuzzy : public TrackerBase
{
public:
    tracker_fuzzy();
    ~tracker_fuzzy();
    virtual bool import_config(const char *file_path);
    virtual bool set_path(const Msg<PathData> &path, const Msg<ControlData> &msg_cur_control_data);
    virtual bool set_goal(const Msg<Pose<FLOAT_T> > &goal);
    virtual Msg<bool> is_goal_reached();
    virtual bool set_stop_signal(const Msg<bool>& stop);
    virtual Msg<ControlData> scroll_calculate(const FLOAT_T &dt, const Msg<Pose<FLOAT_T> > &msg_location, const Msg<ControlData> &msg_cur_control_data);
    bool IsNeedSetPath(const Msg<PathData> &last_path,const Msg<PathData> &current_path);
private:
    void Init();
    std::shared_ptr<FuzzyPID> p_fuzzy_;
    Msg<PathData> last_path_;
    Msg<Pose<FLOAT_T>> goal_;
    bool reached_target_pose_;
    bool is_compute_fuzzy_;
};
}
