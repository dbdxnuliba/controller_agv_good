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
class ServiceStop : public ServiceBase
{
public:
    ServiceStop(const std::string cmd, const std::string arg, const std::string ret):
        ServiceBase(cmd, arg, ret)
    {
        p_task_client = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_TASK));
    }

    void execute_once(void *p_args)
    {
        try
        {
            RetMsg<std::string> ret_msg = std::move(p_task_client->call<RetMsg<std::string>>("set_next_task", TaskId::STOP, std::string("")));
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
