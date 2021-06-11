#include <iostream>
#include <fstream>
#include <mutex>
#include <memory>
#include "common/json.hpp"
#include "common/common.h"
#include "common/thread_rpc.h"
#include "common/data_types.h"
#include "common/time_keeper.h"
#include "common/back_trace.h"
#include "common/msg_id.h"
#include "map/grid_map_data.h"
#include "map/grid_map_interface.h"
#include "modules/main/smoother_main.h"
#include "modules/smoother/gradient_descent_smoother/gradient_descent_smoother.h"
#include "modules/smoother/sample_optimizing_smoother/sample_optimizing_smoother.h"


namespace bz_robot
{
    class SmootherServer
    {
    public:
        std::shared_ptr<thread_rpc::Server> p_server;
        std::string name;
        SmootherServer()
        {
            //mp_smoother = std::make_shared<GradientDescentSmoother>();
            //m_mtx.unlock();
        }

        ReturnStatus import_config(const std::string& config_file)
        {
            RECORD_TIME();
            //std::lock_guard<std::mutex> lock(m_mtx);
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

            if(mp_smoother)
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

//        ReturnStatus init()
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

//        ReturnStatus set_reference_path(const Msg<PathData>& map_path)
//        {
//            p_planner->set_reference_path(map_path.data);
//            return RET_SUCCESS;
//        }

        RetMsg<PathData> smooth(const Msg<GridMapData>& map_data, const Msg<PathData> &ref_path)
        {

            PRINT_DEBUG("{}", name);
            if(!mp_smoother)
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

            return m_ret_msg_path;
        }
        RetMsg<PathData> smoothed_path()
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
        std::shared_ptr<SmootherBase> mp_smoother;
        RetMsg<PathData> m_ret_msg_path;
    };


void smoother_module_main(int argc, char **argv)
{
    using namespace bz_robot;
    PRINT_INFO("init");
    //init_log("smooth_server");
    SmootherServer global_smoother_server;
    global_smoother_server.name = "Global Smoother";
    SmootherServer local_smoother_server;
    local_smoother_server.name = "Local Smoother";

    global_smoother_server.p_server = std::make_shared<thread_rpc::Server>(MACRO_STR(MSG_ID_SERVER_GLOBAL_PATH_SMOOTH), 1);
    local_smoother_server.p_server = std::make_shared<thread_rpc::Server>(MACRO_STR(MSG_ID_SERVER_LOCAL_PATH_SMOOTH), 1);
    global_smoother_server.register_service();
    local_smoother_server.register_service();
    std::thread thread_local_smoother_server(&SmootherServer::run_server, &local_smoother_server);
    thread_local_smoother_server.detach();
    global_smoother_server.run_server();
}
}
