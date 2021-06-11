#pragma once
#include "tracker/tracker_base.h"
#include "common/json.hpp"
#include "model/ackermann_model.h"
#include "model_predictive_control.h"
namespace bz_robot
{
class TrackerParams;
class tracker_mpc : public TrackerBase
{
public:
    tracker_mpc();
    ~tracker_mpc();
    virtual bool import_config(const char *file_path);
    virtual bool set_path(const Msg<PathData> &path, const Msg<ControlData> &msg_cur_control_data);
    virtual bool set_goal(const Msg<Pose<FLOAT_T> > &goal);
    virtual Msg<ControlData> scroll_calculate(const FLOAT_T &dt, const Msg<Pose<FLOAT_T> > &msg_location, const Msg<ControlData> &msg_cur_control_data);
    virtual Msg<bool> is_goal_reached();
    virtual bool set_stop_signal(const Msg<bool>& stop);
    bool IsNeedSetPath(const Msg<PathData> &last_path,const Msg<PathData> &current_path);
    //用于作为计算MPC线程的入口函数
    void ComputeMpc();
    //作为发送控制结果的函数，当MPC控制器没有输出新的计算结果时，利用原有的计算结果的进行插值
    ControlData SetRobotVelocity(const float &dt);
private:
    void Init();
    std::shared_ptr<MPC>  p_mpc_;
    Msg<PathData>         last_path_;
    Msg<Pose<FLOAT_T>>    goal_;
    Msg<Pose<FLOAT_T> >   robot_current_pose_;//机器人当前位置
    Msg<ControlData>      robot_current_vel_;//机器人当前速度
    bool reached_target_pose_;//到达终点
    bool is_compute_mpc_;//true表示需要进行计算，false则关闭线程
    bool has_path_;
    MpcResult mpc_result_;
};
}
