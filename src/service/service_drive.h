#pragma once

#include <vector>

#include "service/service_base.h"
#include "service/service_manager.h"
#include "common/common.h"
#include "common/msg_id.h"
#include "common/thread_rpc.h"
#include "task/task_id.h"

namespace bz_robot
{
class ServiceDrive : public ServiceBase
{
public:
    ServiceDrive(const std::string cmd, const std::string arg, const std::string ret):
        ServiceBase(cmd, arg, ret)
    {
        p_task_client = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_TASK));
    }

    void execute_once(void *p_args)
    {
        try
        {
            nlohmann::json j = nlohmann::json::parse(m_json_msg);
            int trans = j["trans"]; //unit cm/s
            int speed = j["speed"]; //unit %(0-100)
            int rot = j["rot"];     //unit °/s

            FLOAT_T velocity = 1e-2 * trans * speed * 0.01;
            double radian_rot = constraint_angle_r(degree_to_radian(rot), -M_PI, M_PI);
            FLOAT_T steer_angle = radian_rot * speed * 0.01;

            nlohmann::json j_out;
            j_out["#CMD#"] = "DRIVE";
            j_out["velocity"] = velocity;
            j_out["rot"] = steer_angle;
            RetMsg<std::string> ret_msg = std::move(p_task_client->call<RetMsg<std::string>>("set_next_task", TaskId::DRIVE, j_out.dump()));
        }
        catch (...)
        {
            PRINT_ERROR("error");
        }
    }
private:
    std::shared_ptr<thread_rpc::Client> p_task_client;
};
}
