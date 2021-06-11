#pragma once

#include <stdio.h>
#include <sstream>
#include <vector>

#include "service/service_base.h"
#include "service/service_manager.h"
#include "common/time_keeper.h"

namespace bz_robot
{
    class ServiceConnect : public ServiceBase
    {
    public:
        ServiceConnect(const std::string cmd, const std::string arg, const std::string ret):
            ServiceBase(cmd, arg, ret){}

        void execute_once(void *p_args)
        {
            //RECORD_TIME();
            const std::string& argc = m_json_msg;
            //std::cout << argc << std::endl;
            nlohmann::json j_in = nlohmann::json::parse(argc);
            std::vector<const ServiceBase*> service_list = mp_manager->total_service();
            nlohmann::json j_out;
            j_out["#CMD#"] = m_cmd;
            auto commands_value = nlohmann::json::array();
            for(int i = 0; i < service_list.size(); ++i)
            {
                nlohmann::json json_object = nlohmann::json::object();
                json_object["arg"] = service_list[i]->arg();
                json_object["cmd"] = service_list[i]->cmd();
                json_object["ret"] = service_list[i]->ret();
                commands_value.emplace_back(json_object);
            }

            j_out["commands"] = commands_value;
            std::stringstream ss;
            ss << "Connect Succeed: login as " << std::string(j_in["user"]) << ", group contains: " << "all";
            //ss << "Connect Succeed: login as " << "gary" << ", group contains: " << "all";
            j_out["msg"] = ss.str();
            j_out["state"] = true;
            std::string output_str = std::move(j_out.dump());
            //sent to client
            mp_manager->send_msg_to_client(output_str);
            //return true;
        }
    };
}
