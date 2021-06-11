#include "task/task_stop.h"
#include <fstream>
#include "common/msg_id.h"
#include "common/common.h"
#include "common/thread_rpc.h"
#include "common/periodic_task.h"
#include "common/print.h"
#include "common/json.hpp"



namespace bz_robot
{

class TaskStopParams
{
public:
    std::shared_ptr<thread_rpc::Client> p_client_robot;
};

TaskStop::TaskStop()
{
    mp_params = std::make_shared<TaskStopParams>();
    mp_params->p_client_robot = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_ROBOT));
}

TaskStop::~TaskStop()
{

}

RetMsg<std::string> TaskStop::run()
{
	RetMsg<std::string> ret_msg;
	mp_params->p_client_robot->call("stop");
	return ret_msg;
}

}
