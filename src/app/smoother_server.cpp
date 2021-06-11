#include <iostream>
#include <fstream>
#include <mutex>
#include <memory>
#include "common/json.hpp"
#include "third_libs/rest_rpc/include/rest_rpc.hpp"
#include "common/data_types.h"
#include "common/back_trace.h"
#include "common/msg_id.h"
//#include "planner/planner_base.h"
#include "map/grid_map_data.h"
#include "map/grid_map_interface.h"
#include "modules/smoother/gradient_descent_smoother/gradient_descent_smoother.h"
#include "modules/smoother/sample_optimizing_smoother/sample_optimizing_smoother.h"
// #include "planner/ackermann_wave_front_planner/ackermann_wave_front_planner.h"
// #include "planner/frenet_optimal_planner/frenet_optimal_planner.h"

namespace bz_robot
{
    class SmootherServer
    {
    public:
        std::shared_ptr<rest_rpc::rpc_service::rpc_server> p_server;
        std::string name;
        SmootherServer()
        {
            //mp_smoother = std::make_shared<GradientDescentSmoother>();
            m_mtx.unlock();
        }

        ReturnStatus import_config(rpc_conn conn, const std::string config_file)
        {
            std::lock_guard<std::mutex> lock(m_mtx);
            mp_smoother.reset();

            ReturnStatus return_status;
            try
            {
                PRINT_INFO("read config: {}", config_file);
                std::ifstream i(config_file);
                if(i)
                {
                    nlohmann::json j;
                    i >> j;
                    std::string smoother_name = j["NAME"];

                    if(smoother_name == "GRADIENT_DESCENT_SMOOTHER")
                    {
                        mp_smoother = std::make_shared<GradientDescentSmoother>();
                    }
                    else if(smoother_name == "SAMPLE_OPTIMIZER_SMOOTHER")
                    {
                        mp_smoother = std::make_shared<SampleOptimizingSmoother>();
                    }
                    else
                    {
                        //default
                        PRINT_ERROR("smoother type {} error", smoother_name);
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
            if(mp_smoother.get() != nullptr)
            {
                bool status =  mp_smoother->import_config(config_file.c_str());
                if(status)
                {
                    return_status = RET_SUCCESS;
                }
                else
                {
                    return_status = RET_FAILED;
                }
            }
            else
            {
                return_status = RET_SUCCESS;
            }
            return return_status;
        }

//        ReturnStatus init(rpc_conn conn)
//        {
//            ReturnStatus return_status;
//            bool status =  mp_smoother->init();
//            if(status)
//            {
//                return_status = RET_SUCCESS;
//            }
//            else
//            {
//                return_status = RET_FAILED;
//            }
//            return return_status;
//        }

//        ReturnStatus set_reference_path(rpc_conn conn, const Msg<PathData>& map_path)
//        {
//            p_planner->set_reference_path(map_path.data);
//            return RET_SUCCESS;
//        }

        RetMsg<PathData> smooth(rpc_conn conn, const Msg<GridMapData>& map_data, const Msg<PathData> &ref_path)
        {

            PRINT_DEBUG("{}", name);
            if(!m_mtx.try_lock())
            {
                PRINT_WARN("last smooth unfinished");
                RetMsg<PathData> ret_msg;
                ret_msg.return_status = RET_IS_BUSY;
                return ret_msg;
            }

            if(mp_smoother.get() == nullptr)
            {
                m_ret_msg_path.return_status = RET_SUCCESS;
                m_ret_msg_path.msg = std::move(ref_path);
            }
            else
            {
                std::shared_ptr<MapBase> p_map = std::make_shared<GridMapInterface>(map_data.data);
                //PathData &path_data = m_ret_msg_path.msg.data;
                bool status = mp_smoother->smooth(p_map, ref_path.data);
                if(status)
                {
                    m_ret_msg_path.return_status = RET_SUCCESS;
                }
                else
                {
                    m_ret_msg_path.return_status = RET_FAILED;
                }
                m_ret_msg_path.msg.data = std::move(mp_smoother->smoothed_path());
            }
            m_mtx.unlock();
            return m_ret_msg_path;
        }
        RetMsg<PathData> smoothed_path(rpc_conn conn)
        {
            return m_ret_msg_path;
        }

        void register_service()
        {
            p_server->register_handler("import_config", &SmootherServer::import_config, this);
            //p_server->register_handler("init", &SmootherServer::init, this);
            p_server->register_handler("smooth", &SmootherServer::smooth, this);
            p_server->register_handler("smoothed_path", &SmootherServer::smoothed_path, this);
        }

        void run_server()
        {
            PRINT_DEBUG("smoother server running");
            try
            {
                p_server->run();
            }
            catch(...)
            {
                print_trace();
                PRINT_ERROR("error occured\n");
            }
            PRINT_DEBUG("smoother server finished");
        }
    private:
        std::mutex m_mtx;
        std::shared_ptr<SmootherBase> mp_smoother;
        RetMsg<PathData> m_ret_msg_path;
    };
}

int main()
{
    using namespace bz_robot;
    init_log("smooth_server");
    SmootherServer global_smoother_server;
    global_smoother_server.name = "Global Smoother";
    SmootherServer local_smoother_server;
    local_smoother_server.name = "Local Smoother";
    const size_t thread_size = 2;
    global_smoother_server.p_server = std::make_shared<rest_rpc::rpc_service::rpc_server>(MSG_ID_SERVER_GLOBAL_PATH_SMOOTH, thread_size, 0, 0);
    local_smoother_server.p_server = std::make_shared<rest_rpc::rpc_service::rpc_server>(MSG_ID_SERVER_LOCAL_PATH_SMOOTH, thread_size, 0, 0);
    global_smoother_server.register_service();
    local_smoother_server.register_service();
    std::thread thread_local_planner_server(&SmootherServer::run_server, &local_smoother_server);
    thread_local_planner_server.detach();
    global_smoother_server.run_server();
}
