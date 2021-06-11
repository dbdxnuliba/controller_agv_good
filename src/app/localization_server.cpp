#include "modules/localization/localization.h"
#include "common/print.h"
#include "common/common.h"
#include "common/data_types.h"
#include "common/geometry.h"
#include "third_libs/rest_rpc/include/rest_rpc.hpp"
#include "common/msg_id.h"

namespace bz_robot
{
    static std::shared_ptr<Localization> P_LOCALIZATION;
    static std::shared_ptr<rest_rpc::rpc_service::rpc_server> P_SERVER;

    void set_location(rpc_conn conn, const Msg<Pose<FLOAT_T>> &msg_location)
    {
        P_LOCALIZATION->set_location(msg_location);
    }

    RetMsg<Pose<FLOAT_T>> location(rpc_conn conn)
    {
        RetMsg<Pose<FLOAT_T>> ret_msg;
        ret_msg.return_status = RET_SUCCESS;
        ret_msg.msg = P_LOCALIZATION->location();
        return ret_msg;
    }
}

int main()
{
    init_log("localization_server");
    rpc_server server(bz_robot::MSG_ID_SERVER_LOCALIZATION, 1, 0, 0);
    bz_robot::P_LOCALIZATION = std::make_shared<bz_robot::Localization>();

    server.register_handler("set_location", bz_robot::set_location);
    server.register_handler("location", bz_robot::location);

    server.run();
}
