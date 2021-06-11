#include "modules/localization/localization.h"
#include "common/print.h"
#include "common/common.h"
#include "common/data_types.h"
#include "common/geometry.h"
#include "common/thread_rpc.h"
#include "common/msg_id.h"
#include "modules/main/localization_main.h"
namespace bz_robot
{
    static std::shared_ptr<Localization> P_LOCALIZATION;
    // static std::shared_ptr<rest_rpc::rpc_service::rpc_server> P_SERVER;

    static void set_location(const Msg<Pose<FLOAT_T>> &msg_location)
    {
        P_LOCALIZATION->set_location(msg_location);
    }

    static RetMsg<Pose<FLOAT_T>> location()
    {
        RetMsg<Pose<FLOAT_T>> ret_msg;
        ret_msg.return_status = RET_SUCCESS;
        ret_msg.msg = P_LOCALIZATION->location();
        return ret_msg;
    }


void localization_module_main(int argc, char **argv)
{
    using namespace bz_robot;
    PRINT_INFO("init");
    thread_rpc::Server server(MACRO_STR(MSG_ID_SERVER_LOCALIZATION));
    //init_log("localization_server");

    // rpc_server server(bz_robot::MSG_ID_SERVER_LOCALIZATION, 1, 0, 0);

    P_LOCALIZATION = std::make_shared<Localization>();

    server.register_handler("set_location", set_location);
    server.register_handler("location", location);

    server.run();
}

}
