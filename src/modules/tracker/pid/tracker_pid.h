#include "tracker/tracker_base.h"

namespace bz_robot
{
class TrackerParams;
class TrackerPid : public TrackerBase
{
public:
    TrackerPid();
    ~TrackerPid() {}
    virtual bool import_config(const char *file_path);
    virtual bool set_path(const Msg<PathData> &path, const Msg<ControlData> &msg_cur_control_data);
    virtual bool set_goal(const Msg<Pose<FLOAT_T> > &goal);
    virtual Msg<bool> is_goal_reached();
    virtual bool set_stop_signal(const Msg<bool>& stop);
    virtual Msg<ControlData> scroll_calculate(const FLOAT_T &dt, const Msg<Pose<FLOAT_T> > &msg_location, const Msg<ControlData>& msg_cur_control_data);
private:
    std::shared_ptr<TrackerParams> mp_params;
};
}
