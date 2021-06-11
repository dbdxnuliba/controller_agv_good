#pragma once

#include <stdio.h>
#include <sstream>
#include <vector>
#include "common/data_types.h"
#include "task/task_manager.h"
#include "service/service_base.h"
#include "service/service_manager.h"
#include "common/timer/timer.h"
#include "common/common.h"
#include "common/msg_id.h"
#include "common/thread_rpc.h"

namespace bz_robot
{
class ServiceGetPath : public ServiceBase
{
public:
    ServiceGetPath(const std::string cmd, const std::string arg, const std::string ret):
        ServiceBase(cmd, arg, ret)
    {
        mp_task_client = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_TASK));
    }

    void execute_once(void *p_args)
    {
        //PRINT_DEBUG("recv cmd get path");
        PathData global_path = mp_task_client->call<PathData>("global_path");
        const int path_size = global_path.size();
        //PRINT_DEBUG("global path size = {}", path_size);
        if(path_size > 0)
        {
        std::vector<nlohmann::json> json_path;
        json_path.resize(path_size);
        for(int i = 0; i != path_size; ++i)
        {
            json_path[i]["x"] = int(global_path[i].position.x * 1000);
            json_path[i]["y"] = int(global_path[i].position.y * 1000);
        }
        nlohmann::json j_out;
        j_out["#CMD#"] = m_cmd;
        j_out["num"] = global_path.size();
        j_out["points"] = json_path;

        std::string out_put_str  = j_out.dump();

        //PRINT_DEBUG("path data:\n", j_out.dump(4));
        mp_manager->send_msg_to_client(out_put_str);
        }
    }
private:
    std::shared_ptr<thread_rpc::Client> mp_task_client;
};

}
