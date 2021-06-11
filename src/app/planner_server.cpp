#include <iostream>
#include <fstream>
#include <mutex>
#include <memory>
#include "common/json.hpp"
#include "third_libs/rest_rpc/include/rest_rpc.hpp"
#include "common/data_types.h"
#include "common/back_trace.h"
#include "common/msg_id.h"
#include "planner/planner_base.h"
#include "map/grid_map_data.h"
#include "map/grid_map_interface.h"
#include "planner/ackermann_wave_front_planner/ackermann_wave_front_planner.h"
#include "planner/frenet_optimal_planner/frenet_optimal_planner.h"
#include "planner/r_star_planner/r_star_planner.h"

namespace bz_robot
{
    static void clear_robot_cur_pose_obstacles(std::shared_ptr<MapBase> p_map, const Pose<float> &pose_robot)
    {
        //RECORD_TIME();
        std::vector<VectorX2<float>> footprint = p_map->robot_footprint();
        float min_x = DBL_MAX;
        float max_x = 0;
        float min_y = DBL_MAX;
        float max_y = 0;

        float cos_heading_angle = cos(pose_robot.heading_angle);
        float sin_heading_angle = sin(pose_robot.heading_angle);
        float map_pose_x = 0;
        float map_pose_y = 0;
        p_map->world_to_map(pose_robot.position.x, pose_robot.position.y, &map_pose_x, &map_pose_y);
        for(int i = 0; i < footprint.size(); i++)
        {
            VectorX2<float> new_pt;
            new_pt.x = (float)map_pose_x + (footprint[i].x * cos_heading_angle - footprint[i].y * sin_heading_angle)/p_map->resolution();
            new_pt.y = (float)map_pose_y + (footprint[i].x * sin_heading_angle + footprint[i].y * cos_heading_angle)/p_map->resolution();
            footprint[i] = new_pt;
            min_x = std::min(min_x, footprint[i].x);
            max_x = std::max(max_x, footprint[i].x);
            min_y = std::min(min_y, footprint[i].y);
            max_y = std::max(max_y, footprint[i].y);
        }

        uint32_t range_min_x = uint32_t(std::max(min_x, 0.0f));
        uint32_t range_max_x = uint32_t(std::min(uint32_t(max_x), p_map->size_in_cells_x() - 1));
        uint32_t range_min_y = uint32_t(std::max(min_y, 0.0f));
        uint32_t range_max_y = uint32_t(std::min(uint32_t(max_y), p_map->size_in_cells_y() - 1));

        for(uint32_t x_pos = range_min_x; x_pos <= range_max_x; ++x_pos)
        {
            for(uint32_t y_pos = range_min_y; y_pos <= range_max_y; ++y_pos)
            {
                VectorX2<float> world_pos;
                p_map->map_to_world(x_pos, y_pos, &world_pos.x, &world_pos.y);
                VectorX2<float> point((float(x_pos)), float (y_pos));
                if(p_map->is_inside_robot_footprint(point, footprint))
                {
                    p_map->set_cost(x_pos, y_pos, 0);
                }
            }
        }
    }
    class PlannerServer
    {
    public:
        std::shared_ptr<PlannerBase> p_planner;
        RetMsg<PathData> m_ret_msg_path;
        bool is_enable_pub = true;
        std::shared_ptr<rest_rpc::rpc_service::rpc_server> p_server;
        std::string planner_name;
        PlannerServer()
        {
            //default
            p_planner = std::make_shared<ackermann_wave_front_planner::WaveFrontPlanner>();
            m_mtx.unlock();
        }

        ReturnStatus import_config(rpc_conn conn, const std::string config_file)
        {
            ReturnStatus return_status;
            std::string name;
            try
            {
                PRINT_INFO("read config: {}", config_file);
                std::ifstream i(config_file);
                if(i)
                {
                    nlohmann::json j;
                    i >> j;
                    name = j["PLANNER_NAME"];

                    if(name == "ACKERMANN_WAVE_FRONT")
                    {
                        p_planner = std::make_shared<ackermann_wave_front_planner::WaveFrontPlanner>();
                    }
                    else if(name == "FRENET_OPTIMAL")
                    {
                        p_planner = std::make_shared<FrenetOptimalPlanner>();
                    }
                    else if(name == "R_STAR")
                    {
                        p_planner = std::make_shared<r_star_planner::RStarPlanner>();
                    }
                    else
                    {
                        PRINT_ERROR("planner type {} error", name);
                    }
                }
                else
                {
                    PRINT_ERROR("can't read config files from: {}\n", config_file);
                }
            }
            catch(std::exception& e )
            {
                PRINT_ERROR("{}\n", e.what());
            }
            catch(...)
            {
                PRINT_ERROR("un expexted occured\n");
            }

            bool status =  p_planner->import_config(config_file.c_str());
            if(status)
            {
                return_status = RET_SUCCESS;
            }
            else
            {
                return_status = RET_FAILED;
            }
            return return_status;
        }

        ReturnStatus init(rpc_conn conn)
        {
            ReturnStatus return_status;
            bool status =  p_planner->init();
            if(status)
            {
                return_status = RET_SUCCESS;
            }
            else
            {
                return_status = RET_FAILED;
            }
            return return_status;
        }

        ReturnStatus set_reference_path(rpc_conn conn, const Msg<PathData>& map_path)
        {
            p_planner->set_reference_path(map_path.data);
            return RET_SUCCESS;
        }

        RetMsg<PathData> plan(rpc_conn conn, const Msg<GridMapData>& map_data,
                                     const Msg<Pose<FLOAT_T>> &pose_start,const Msg<Pose<FLOAT_T>> &pose_goal)
        {
            //PRINT_DEBUG("{}", planner_name);
            if(!m_mtx.try_lock())
            {
                PRINT_WARN("last planning unfinished");
                RetMsg<PathData> ret_msg;
                ret_msg.return_status = RET_IS_BUSY;
                return ret_msg;
            }
            std::shared_ptr<MapBase> p_map = std::make_shared<GridMapInterface>(map_data.data);
            clear_robot_cur_pose_obstacles(p_map, pose_start.data);
            PathData &path_data = m_ret_msg_path.msg.data;
            bool status = p_planner->plan(p_map, pose_start.data, pose_goal.data, &path_data);
            if(status)
            {
                m_ret_msg_path.return_status = RET_SUCCESS;
            }
            else
            {
                m_ret_msg_path.return_status = RET_FAILED;
            }
            if(is_enable_pub)
            {
                p_server->publish("path", m_ret_msg_path);
            }
            m_mtx.unlock();
            return m_ret_msg_path;
        }

        RetMsg<PathData> path(rpc_conn conn)
        {
            return m_ret_msg_path;
        }
        ReturnStatus reset(rpc_conn conn)
        {
            p_planner->reset();
            return RET_SUCCESS;
        }

        ReturnStatus stop(rpc_conn conn)
        {
            p_planner->stop();
            return RET_SUCCESS;
        }

        ReturnStatus is_stop(rpc_conn conn)
        {
            ReturnStatus return_status;
            if(p_planner->is_stop())
            {
                return_status = RET_SUCCESS;
            }
            else
            {
                return_status = RET_FAILED;
            }
            return return_status;
        }

        void register_service()
        {
            p_server->register_handler("import_config", &PlannerServer::import_config, this);
            p_server->register_handler("init", &PlannerServer::init, this);
            p_server->register_handler("set_reference_path", &PlannerServer::set_reference_path, this);
            p_server->register_handler("plan", &PlannerServer::plan, this);
            p_server->register_handler("path", &PlannerServer::path, this);
            p_server->register_handler("reset", &PlannerServer::reset, this);
            p_server->register_handler("stop", &PlannerServer::stop, this);
            p_server->register_handler("is_stop", &PlannerServer::is_stop, this);
        }

        void run_server()
        {
            PRINT_DEBUG("planner server running");
            try
            {
                p_server->run();
            }
            catch(...)
            {
                print_trace();
                PRINT_ERROR("error occured\n");
            }
            PRINT_DEBUG("planner server finished");
        }
    private:
        std::mutex m_mtx;
    };
}

int main()
{
    using namespace bz_robot;
    init_log("planner_server");
    PlannerServer global_planner_server;
    global_planner_server.planner_name = "Global Planner";
    PlannerServer local_planner_server;
    local_planner_server.planner_name = "Local Planner";
    const size_t thread_size = 2;
    global_planner_server.p_server = std::make_shared<rest_rpc::rpc_service::rpc_server>(MSG_ID_SERVER_GLOBAL_PLAN, thread_size, 0, 0);
    local_planner_server.p_server = std::make_shared<rest_rpc::rpc_service::rpc_server>(MSG_ID_SERVER_LOCAL_PLAN, thread_size, 0, 0);
    global_planner_server.register_service();
    local_planner_server.register_service();
    std::thread thread_local_planner_server(&PlannerServer::run_server, &local_planner_server);
    thread_local_planner_server.detach();

    global_planner_server.run_server();
}
