#pragma once

#include <stdio.h>
#include <sstream>
#include <vector>
#include "common/data_types.h"
#include "task/task_manager.h"
#include "service/service_base.h"
#include "service/service_manager.h"
#include "common/timer/timer.h"
#include "common/thread_rpc.h"
#include "common/common.h"
#include "common/msg_id.h"


namespace bz_robot
{
class ServiceGetRobotInfo : public ServiceBase
{
public:
    ServiceGetRobotInfo(const std::string cmd, const std::string arg, const std::string ret):
        ServiceBase(cmd, arg, ret)
    {
        mp_task_client = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_TASK));
    }

    void execute_once(void *p_args)
    {
        RobotInfo robot_info = mp_task_client->call<RobotInfo>("robot_info");
        nlohmann::json j_out;
        j_out["#CMD#"] = m_cmd;
        j_out["x"] = int(robot_info.pose.position.x * 1000);
        j_out["y"] = int(robot_info.pose.position.y * 1000);
        j_out["th"] = int(radian_to_degree(robot_info.pose.heading_angle));
        j_out["vel_f"] = double(robot_info.control_data.velocity);
        j_out["vel_r"] = double(robot_info.control_data.velocity);
        j_out["battery"] = robot_info.battery;
        std::string out_put_str  = j_out.dump();
        //sent to client
        mp_manager->send_msg_to_client(out_put_str);
    }
private:
    std::shared_ptr<thread_rpc::Client> mp_task_client;
};
}
