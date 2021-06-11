#include "common/print.h"
#include "common/common.h"
#include "common/data_types.h"
#include "common/geometry.h"
#include "common/msg_id.h"
#include "common/thread_rpc.h"
#include "modules/main/task_main.h"
#include "task/task_manager.h"
#include "task/task_id.h"
#include "task/task_query_status.h"


namespace bz_robot
{
    static std::shared_ptr<TaskManager> P_TASK_MANAGER;
    static TaskQueryStatus TASK_QUERY_INTERFACE;
    //static std::shared_ptr<rest_rpc::rpc_service::rpc_server> P_SERVER;

    static ReturnStatus import_config(const std::string& config_file)
    {
        ReturnStatus return_status;
        bool status = P_TASK_MANAGER->import_configs(config_file);
        return_status = ((status == true)? RET_SUCCESS : RET_ERROR);
        return return_status;
    }

    static bool start()
    {
        return P_TASK_MANAGER->start();
    }

    static bool set_next_task(const TaskId &task_id, const std::string& argc)
    {
        return P_TASK_MANAGER->set_next_task(task_id, argc);
    }

    static bool stop_task(const TaskId &task_id)
    {
        return P_TASK_MANAGER->stop_task(task_id);
    }

    static RetMsg<std::string> run_task(const TaskId &task_id, const std::string& argc)
    {
        return P_TASK_MANAGER->run_task(task_id, argc);
    }

    void task_module_main(int argc, char **argv)
    {
        //init_log("task_server");
        P_TASK_MANAGER = std::make_shared<TaskManager>();
        thread_rpc::Server server(MACRO_STR(MSG_ID_SERVER_TASK), 2);
//        PRINT_INFO("init");
//        import_config(argv[1]);
//        start();

        server.register_handler("import_config", import_config);
        server.register_handler("task_module_start", start);
        server.register_handler("set_next_task", set_next_task);
        server.register_handler("stop_task", stop_task, 1);
        server.register_handler("run_task", run_task, 1);

        server.register_handler("localization_status", &TaskQueryStatus::localization_status, &TASK_QUERY_INTERFACE);
        server.register_handler("safe_drive_status", &TaskQueryStatus::safe_drive_status, &TASK_QUERY_INTERFACE);
        server.register_handler("robot_name", &TaskQueryStatus::robot_name, &TASK_QUERY_INTERFACE);
        server.register_handler("map_name", &TaskQueryStatus::map_name, &TASK_QUERY_INTERFACE);
        server.register_handler("global_map", &TaskQueryStatus::global_map, &TASK_QUERY_INTERFACE);
        server.register_handler("global_path", &TaskQueryStatus::global_path, &TASK_QUERY_INTERFACE);
        server.register_handler("local_path", &TaskQueryStatus::local_path, &TASK_QUERY_INTERFACE);
        server.register_handler("motor_state", &TaskQueryStatus::motor_state, &TASK_QUERY_INTERFACE);
        server.register_handler("robot_info", &TaskQueryStatus::robot_info, &TASK_QUERY_INTERFACE);
        server.run();
    }
}
