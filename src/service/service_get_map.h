#pragma once

#include <stdio.h>
#include <sstream>
#include <vector>

#include "service/service_base.h"
#include "service/service_manager.h"
#include "common/data_types.h"
#include "common/common.h"
#include "common/thread_rpc.h"
#include "common/msg_id.h"
#include "common/time_keeper.h"
#include "task/task_id.h"
#include "map/grid_map_interface.h"


namespace bz_robot
{
class ServiceGetMap : public ServiceBase
{
public:
    ServiceGetMap(const std::string cmd, const std::string arg, const std::string ret):
        ServiceBase(cmd, arg, ret)
    {
        mp_task_client = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_TASK));
    }

    void execute_once(void *p_args)
    {
        RECORD_TIME();
        lock();

        GridMapData grid_map_data = mp_task_client->call<GridMapData>("global_map");
        GridMapInterface grid_map(grid_map_data);
        uint32_t num_points = 0;
        const std::vector<std::vector<int8_t>> map_data = std::move(grid_map.map_data());
        const int obstacle_value = grid_map.obstacles_cost();
        std::vector<std::vector<int>> json_map_data;
        const uint32_t x_size = map_data.size();
        const int origin_x = grid_map.origin_x() * 1000;
        const int origin_y = grid_map.origin_y() * 1000;
        const uint32_t resolution = grid_map.resolution() * 1000;
        int min_x = origin_x;
        int min_y = origin_y;
        int max_x = (grid_map.origin_x() + grid_map.size_in_meters_x()) * 1000;
        int max_y = (grid_map.origin_y() + grid_map.size_in_meters_y()) * 1000;
        for(uint32_t x = 0; x != x_size; ++x)
        {
            bool is_exist_obstacles = false;
            std::vector<int> map_x_data;
            const uint32_t y_size = map_data[x].size();
            map_x_data.emplace_back(origin_x + x * resolution);
            for(uint32_t y = 0; y != y_size; ++y)
            {
                if(map_data[x][y] == obstacle_value)
                {
                    ++num_points;
                    is_exist_obstacles = true;
                    map_x_data.emplace_back(origin_y + y * resolution);
                }
            }
            if(is_exist_obstacles)
            {
                json_map_data.emplace_back(map_x_data);
            }
        }

        //nlohmann::json j_out;
        nlohmann::json j_out;
        j_out["#CMD#"] = m_cmd;
        j_out["Header"] = "";
        j_out["MapRes"] = int(resolution);

        char str[100];
        sprintf(str, "%d %d", max_x, max_y);
        j_out["MaxPose"] = str;
        sprintf(str, "%d %d", min_x, min_y);
        j_out["MinPose"] = str;
        j_out["NumLines"] = 0;
        j_out["NumPoints"] = num_points;
        j_out["ObsPoints"] = json_map_data;
        std::string um_map = std::move(j_out.dump());
        //PRINT_DEBUG("um map data size = {}", um_map.size());
        {
            //RECORD_TIME("send map");
        mp_manager->send_msg_to_client(um_map);
        }
        unlock();
    }
private:
    std::shared_ptr<thread_rpc::Client> mp_task_client;
};
}
