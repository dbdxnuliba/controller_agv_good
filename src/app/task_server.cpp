#include "common/print.h"
#include "common/common.h"
#include "common/data_types.h"
#include "common/geometry.h"
#include "common/msg_id.h"
#include "third_libs/rest_rpc/include/rest_rpc.hpp"
#include "task/task_manager.h"

namespace bz_robot
{
    static std::shared_ptr<TaskManager> P_TASK_MANAGER;
    //static std::shared_ptr<rest_rpc::rpc_service::rpc_server> P_SERVER;

    ReturnStatus import_config(rpc_conn conn, const std::string config_file)
    {
        ReturnStatus return_status;
        bool status = P_TASK_MANAGER->import_configs(config_file);
        return_status = ((status == true)? RET_SUCCESS : RET_ERROR);
        return return_status;
    }
}

int main()
{
    init_log("task_server");
    rpc_server server(bz_robot::MSG_ID_SERVER_TASK, 2, 0, 0);
    bz_robot::P_TASK_MANAGER = std::make_shared<bz_robot::TaskManager>();

//    server.register_handler("set_location", bz_robot::set_location);
//    server.register_handler("location", bz_robot::location);

    server.run();
}
