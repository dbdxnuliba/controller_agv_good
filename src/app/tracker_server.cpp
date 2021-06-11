#include <iostream>
#include <fstream>
#include <mutex>
#include <memory>
#include "common/json.hpp"
#include "third_libs/rest_rpc/include/rest_rpc.hpp"
#include "common/data_types.h"
#include "common/back_trace.h"
#include "common/print.h"
#include "common/msg_id.h"
#include "tracker/tracker_base.h"
#include "tracker/pid/tracker_pid.h"
#include "tracker/fuzzy/tracker_fuzzy.h"
#include "tracker/lqr/tracker_lqr.h"
#include "tracker/mpc/tracker_mpc.h"

#include "common/back_trace.h"
namespace bz_robot
{
    std::shared_ptr<TrackerBase> P_TRACKER = std::make_shared<TrackerPid>();
    ReturnStatus import_config(rpc_conn conn, const std::string config_file)
    {
        ReturnStatus return_status = RET_FAILED;
        try
        {
            std::ifstream i(config_file);
            if(i)
            {
                std::cout<<"控制器参数文件读取成功"<<std::endl;
                nlohmann::json j;
                i >> j;
                std::cout << j << std::endl;
                std::string tracker_name = j["TRACKER_NAME"];

                if(tracker_name == "PID")
                {
                  P_TRACKER = std::make_shared<TrackerPid>();
                }
                else if(tracker_name == "LQR")
                {
                  P_TRACKER = std::make_shared<tracker_lqr>();
                }
                else if(tracker_name == "MPC")
                {
                   P_TRACKER = std::make_shared<tracker_mpc>();
                }
                else if(tracker_name == "FUZZY")
                {
                  P_TRACKER = std::make_shared<tracker_fuzzy>();
                }
                else
                {
                    //default
                    P_TRACKER = std::make_shared<TrackerPid>();
                }
                return_status = RET_SUCCESS;
            }
            else
            {
                PRINT_ERROR("can't read config files from: {}\n", config_file);
            }
        }
        catch(std::exception& e )
        {
            PRINT_ERROR("{}\n", e.what());
        }
        catch(...)
        {
            PRINT_ERROR("un expexted occured\n");
        }

        if(return_status == RET_SUCCESS)
        {
            bool status =  P_TRACKER->import_config(config_file.c_str());
            if(status)
            {
                return_status = RET_SUCCESS;
            }
            else
            {
                return_status = RET_FAILED;
            }
        }

        return return_status;
    }

    ReturnStatus init(rpc_conn conn)
    {
        ReturnStatus return_status = RET_FAILED;
        bool status = P_TRACKER->init();
        if(status)
        {
            return_status = RET_SUCCESS;
        }
        return return_status;
    }

    ReturnStatus set_path(rpc_conn conn, const Msg<PathData> &path, const Msg<ControlData> &msg_control_data)
    {
        ReturnStatus return_status = RET_FAILED;
        bool status = P_TRACKER->set_path(path, msg_control_data);
        if(status)
        {
            return_status = RET_SUCCESS;
        }
        return return_status;
    }

    ReturnStatus set_goal(rpc_conn conn, const Msg<Pose<FLOAT_T> > &goal)
    {
        ReturnStatus return_status = RET_FAILED;
        bool status = P_TRACKER->set_goal(goal);
        if(status)
        {
            return_status = RET_SUCCESS;
        }
        return return_status;
    }

    RetMsg<ControlData> scroll_calculate(rpc_conn conn, const FLOAT_T &dt, const Msg<Pose<FLOAT_T> > &msg_location,
                                         const Msg<ControlData>& msg_cur_control_data)
    {
        RetMsg<ControlData> ret_msg;
        ret_msg.return_status = RET_SUCCESS;
        ret_msg.msg = std::move(P_TRACKER->scroll_calculate(dt, msg_location, msg_cur_control_data));
        return ret_msg;
    }
    //控制器认为已经超出目标点范围，上报客户端
    RetMsg<bool> goal_reached(rpc_conn conn)
    {
      RetMsg<bool> ret_msg;
      ret_msg.return_status = RET_SUCCESS;
      ret_msg.msg =std::move(P_TRACKER->is_goal_reached());
      return ret_msg;
    }
    //接受是否停止计算的信号，类似MPC控制器，需要的计算周期长，需要单独开启一个线程
    ReturnStatus set_stop_signal(rpc_conn conn, const Msg<bool> &stop)
    {
      ReturnStatus return_status = RET_FAILED;
      bool status = P_TRACKER->set_stop_signal(stop);
      if(status)
      {
          return_status = RET_SUCCESS;
      }
      return return_status;
    }
}

int main(int argc, char **argv)
{
    using namespace bz_robot;
    init_log("tracker_server");
    ros::init(argc,argv,"tracker_server");
    //default
    P_TRACKER = std::make_shared<TrackerPid>();
    rest_rpc::rpc_service::rpc_server server(MSG_ID_SERVER_TRACKER, 1, 0, 0);
    std::cout<<"××××××启动路径跟踪服务器××××××"<<std::endl;
    server.register_handler("import_config", import_config);
    server.register_handler("set_path", set_path);
    server.register_handler("set_goal", set_goal);
    server.register_handler("scroll_calculate", scroll_calculate);
    server.register_handler("set_stop_signal", set_stop_signal);
    server.register_handler("is_goal_reached", goal_reached);
    server.run();
    return 0;
}
