#pragma once

#include <vector>

#include "service/service_base.h"
#include "service/service_manager.h"
#include "common/print.h"
#include "common/common.h"
#include "common/data_types.h"
#include "common/thread_rpc.h"
#include "common/msg_id.h"
#include "task_manager.h"

namespace bz_robot
{
class ServiceReconfig : public ServiceBase
{
public:
    ServiceReconfig(const std::string cmd, const std::string arg, const std::string ret):
        ServiceBase(cmd, arg, ret)
    {
        p_task_client = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_TASK));
    }

    void execute_once(void *p_args)
    {
        const std::string& argc = m_json_msg;
        nlohmann::json j = nlohmann::json::parse(argc);
        std::string config_file = j["config_file"];
        p_task_client->call<ReturnStatus>("import_configs", config_file);
    }
private:
    std::shared_ptr<thread_rpc::Client> p_task_client;
};
}
