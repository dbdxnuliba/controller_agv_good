#pragma once

#include <vector>

#include "service/service_base.h"
#include "service/service_manager.h"

namespace bz_robot
{
class ServiceDock : public ServiceBase
{
public:
    ServiceDock(const std::string cmd, const std::string arg, const std::string ret):
        ServiceBase(cmd, arg, ret){}

    void execute_once(void *p_args)
    {
    }
};
}
