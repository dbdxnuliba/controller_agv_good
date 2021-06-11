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
class ServiceGetSafeDrive : public ServiceBase
{
public:
    ServiceGetSafeDrive(const std::string cmd, const std::string arg, const std::string ret):
        ServiceBase(cmd, arg, ret)
    {
        mp_task_client = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_TASK));
    }

    void execute_once(void *p_args)
    {
        nlohmann::json j_out;
        j_out["#CMD#"] = m_cmd;
        j_out["flag"] = mp_task_client->call<bool>("safe_drive_status");
        std::string out_put_str  = j_out.dump();
        //sent to client
        mp_manager->send_msg_to_client(out_put_str);
    }
private:
    std::shared_ptr<thread_rpc::Client> mp_task_client;
};

}
