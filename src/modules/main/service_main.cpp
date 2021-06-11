#include <iostream>
#include <chrono>
#include <boost/asio.hpp>
#include "service/service_manager.h"
#include "service/service_base.h"
#include "service/service_connect.h"
#include "service/service_dock.h"
#include "service/service_drive.h"
#include "service/service_goto.h"
#include "service/service_get_map.h"
#include "service/service_get_map_name.h"
#include "service/service_get_name.h"
#include "service/service_reconfig.h"
#include "service/service_get_loc_state.h"
#include "service/service_get_robot_info.h"
#include "service/service_get_safe_drive.h"
#include "service/service_get_path.h"
#include "service/service_stop.h"
#include "task/task_manager.h"
#include "common/back_trace.h"
#include "modules/main/service_main.h"


namespace bz_robot
{
    static std::string deal_msg(std::shared_ptr<ServiceManager> p_service_manager, const std::string& data)
    {
        std::string cmd;
        nlohmann::json j;
        try
        {
            j = nlohmann::json::parse(data);
            cmd = j["#CMD#"];
        }
        catch (...)
        {
            PRINT_ERROR("can't find [#CMD#], msg:\n{}", j.dump(4));
        }
        p_service_manager->set_next_service(j["#CMD#"], data);
        std::string str_result;
        return str_result;
    }


    static void client_session(std::shared_ptr<boost::asio::ip::tcp::socket> p_socket)
    {
        std::shared_ptr<ServiceManager> p_service_manager = std::make_shared<ServiceManager>();
        std::shared_ptr<ServiceBase> p_service_connect = std::make_shared<ServiceConnect>("UmConnect", "none", "");
        std::shared_ptr<ServiceBase> p_service_dock = std::make_shared<ServiceDock>("UmDock", "none", "");
        std::shared_ptr<ServiceBase> p_service_drive = std::make_shared<ServiceDrive>("UmDrive", "none", "");
        std::shared_ptr<ServiceBase> p_service_goto = std::make_shared<ServiceGoto>("UmGoto", "none", "");
        std::shared_ptr<ServiceBase> p_service_get_map = std::make_shared<ServiceGetMap>("UmGetMap", "none", "Type,Objs,Lines,Points,Resolution,NumPoints");
        std::shared_ptr<ServiceBase> p_service_get_map_name = std::make_shared<ServiceGetMapName>("UmGetMapName", "none", "name");
        std::shared_ptr<ServiceBase> p_service_get_name = std::make_shared<ServiceGetName>("UmGetName", "none", "name");
        std::shared_ptr<ServiceBase> p_service_get_loc_state = std::make_shared<ServiceGetLocState>("UmGetLocState", "none", "name");
        std::shared_ptr<ServiceBase> p_service_get_path = std::make_shared<ServiceGetPath>("UmGetPath", "none", "");
        std::shared_ptr<ServiceBase> p_service_get_robot_info = std::make_shared<ServiceGetRobotInfo>("UmGetRobotInfo", "none", "name");
        std::shared_ptr<ServiceBase> p_service_get_safe_drive = std::make_shared<ServiceGetSafeDrive>("UmGetSafeDrive", "none", "flag");
        std::shared_ptr<ServiceBase> p_service_stop = std::make_shared<ServiceStop>("UmStop", "none", "flag");
        std::shared_ptr<ServiceBase> p_service_reconfig = std::make_shared<ServiceReconfig>("BZRobotReconfig", "none", "");

        p_service_manager->register_service(p_service_connect);
        p_service_manager->register_service(p_service_dock);
        p_service_manager->register_service(p_service_drive);
        p_service_manager->register_service(p_service_goto);
        p_service_manager->register_service(p_service_get_map);
        p_service_manager->register_service(p_service_get_map_name);
        p_service_manager->register_service(p_service_get_name);
        p_service_manager->register_service(p_service_get_loc_state);
        p_service_manager->register_service(p_service_get_path);
        p_service_manager->register_service(p_service_get_robot_info);
        p_service_manager->register_service(p_service_get_safe_drive);
        p_service_manager->register_service(p_service_stop);
        p_service_manager->register_service(p_service_reconfig);

        p_service_manager->set_client_socket(p_socket);

        int header_flag1 = -1;
        int header_flag2 = -1;
        int split_flag1 = -1;
        int split_flag2 = -1;
        int tail_flag1 = -1;
        int tail_flag2 = -1;
        std::string str_data_len;
        std::string str_data;
        uint32_t data_len = 0;
        uint32_t recv_data_len = 0;
        std::string msg;
    //    //此大小决定了tcp最多能读到多少数据
        msg.resize(1024);

        try
        {
            while(true)
            {

                // // 阻塞发送作者名称到客户端
                // socket.write_some(boost::asio::buffer("hello CSND_Ayo"));
                // 阻塞接收客户端发来的数据

                size_t msg_size = 0;

                char buf_data[1024];
                memset(buf_data, 0, 1024);
                boost::system::error_code ec;
                msg_size = p_socket->read_some(boost::asio::buffer(buf_data, 1024), ec);
                msg = std::string(buf_data);
                if(ec)
                {
                    PRINT_ERROR("read error: {}", boost::system::system_error(ec).what());
                    break;
                }
                if(msg_size == 0)
                {
                    PRINT_ERROR("read data = 0");
                    break;
                }
                //std::lock_guard<std::recursive_mutex> lock(* (p_service_manager->asio_mtx()));
                //const size_t msg_size = socket.read_some(boost::asio::buffer(msg));
                //boost::asio::streambuf sbuf;
                //const size_t n = boost::asio::read(socket, boost::asio::buffer(buf));

    //            printf("n = \n%s\n", n, buf);

                //PRINT_DEBUG("msg size = %d\n", msg_size);
                PRINT_DEBUG("recv msg:\n{}\n", msg.c_str());

                //continue;
                for(size_t i = 0; i < msg_size; ++i)
                {
                    //deal header
                    if(header_flag1 == -1 || header_flag2 == -1)
                    {
                        if(msg.data()[i] == '$')
                        {
                            header_flag1 = i;
                            continue;
                        }
                        if(msg.data()[i] == '#')
                        {
                            header_flag2 = i;
                            if(header_flag2 == 0 || header_flag2 == header_flag1 + 1)
                            {
                                str_data_len.clear();
                                str_data.clear();
                                data_len = 0;
                                recv_data_len = 0;
                                continue;
                            }
                            else
                            {
                                //header errror
                                PRINT_ERROR("heading error, i = {}", i);
                                header_flag1 = -1;
                                header_flag2 = -1;
                                split_flag1 = -1;
                                split_flag2 = -1;
                                tail_flag1 = -1;
                                tail_flag2 = -1;
                                str_data_len.clear();
                                str_data.clear();
                                data_len = 0;
                                recv_data_len = 0;
                                continue;
                            }
                        }
                    }
                    //deal data_len
                    if(header_flag1 != -1 && header_flag2 != -1 && (split_flag1 == -1 || split_flag2 == -1))
                    {
                        if(msg.data()[i] == '#' && split_flag1 == -1)
                        {
                            split_flag1 = i;
                            continue;
                        }
                        if(msg.data()[i] == '#' && split_flag1 != -1)
                        {
                            split_flag2 = i;
                            if(split_flag2 == 0 || split_flag2 == split_flag1 + 1)
                            {
                                data_len = atol(str_data_len.c_str());
                                str_data_len.clear();
                                str_data.clear();
                                continue;
                            }
                            else
                            {
                                PRINT_ERROR("data len error, i = {}", i);
                                header_flag1 = -1;
                                header_flag2 = -1;
                                split_flag1 = -1;
                                split_flag2 = -1;
                                tail_flag1 = -1;
                                tail_flag2 = -1;
                                str_data_len.clear();
                                str_data.clear();
                                data_len = 0;
                                recv_data_len = 0;
                                continue;
                            }
                        }
                        //recv correct header
                        str_data_len.push_back(msg.data()[i]);
                    }
                    //deal_len
                    if(header_flag1 != -1 && header_flag2 != -1 &&
                        split_flag1 != -1 && split_flag2 != -1 &&
                        recv_data_len != data_len &&
                        (tail_flag1 == -1 && tail_flag2 == -1))
                    {
                        int need_recv_data_len = data_len - recv_data_len;
                        if(need_recv_data_len > msg_size - i)
                        {
                            str_data.insert(str_data.size(), msg.substr(i, msg_size));
                            recv_data_len += msg_size - i;
                            i = msg_size - 1;
                            continue;
                        }
                        else
                        {
                            str_data.insert(str_data.size(), msg.substr(i, need_recv_data_len).c_str());
                            recv_data_len += need_recv_data_len;
                            i = i + need_recv_data_len - 1;
                            continue;
                        }
                    }
                    if(header_flag1 != -1 && header_flag2 != -1 &&
                        split_flag1 != -1 && split_flag2 != -1 &&
                        recv_data_len == data_len &&
                        (tail_flag1 == -1 || tail_flag2 == -1))
                    {
                        if(msg.data()[i] == '$' && tail_flag1 == -1)
                        {
                            tail_flag1 = i;
                            continue;
                        }
                        if(msg.data()[i] == '~' && tail_flag1 != -1)
                        {
                            tail_flag2 = i;
                            if(tail_flag2 == 0 || tail_flag2 == tail_flag1 + 1)
                            {
                                //deal msg
                                PRINT_DEBUG("deal single msg:\n{}", str_data.c_str());
                                bz_robot::deal_msg(p_service_manager, str_data);
                                //PRINT_DEBUG("msg index = {}", i);
                                //reset flags
                                header_flag1 = -1;
                                header_flag2 = -1;
                                split_flag1 = -1;
                                split_flag2 = -1;
                                tail_flag1 = -1;
                                tail_flag2 = -1;
                                str_data_len.clear();
                                str_data.clear();
                                data_len = 0;
                                recv_data_len = 0;
                            }
                            else
                            {
                                //tail error
                                PRINT_ERROR("tail error, i = {}", i);
                                header_flag1 = -1;
                                header_flag2 = -1;
                                split_flag1 = -1;
                                split_flag2 = -1;
                                tail_flag1 = -1;
                                tail_flag2 = -1;
                                str_data_len.clear();
                                str_data.clear();
                                data_len = 0;
                                recv_data_len = 0;
                                continue;
                            }
                        }
                    }
                }
                //PRINT_DEBUG("finish deal msg");
            }
        }
        catch(std::exception& e )
        {
            PRINT_ERROR("exception: {}\n", e.what());
        }
        catch (...)
        {
            PRINT_ERROR("error occured");
        }
        p_service_manager->stop();
        PRINT_INFO("client session exit!\n");
    }

    static void serivce_running()
    {
        // asio程序必须的io_service对象
        boost::asio::io_service ios;
        // 具体的服务器地址与端口
        boost::asio::ip::tcp::endpoint endpotion(boost::asio::ip::tcp::v4(), 7273);
        // 创建acceptor对象，当前的IPV4作为服务器地址(127.0.0.1 || 0.0.0.0)，接受端口7273的消息.
        boost::asio::ip::tcp::acceptor acceptor(ios, endpotion);

        PRINT_INFO("addr: {}", acceptor.local_endpoint().address().to_string());
        PRINT_INFO("port: {}", acceptor.local_endpoint().port());

        while(true)
        {
            PRINT_INFO("waiting connection ...");
            std::shared_ptr<boost::asio::ip::tcp::socket> p_socket = std::make_shared<boost::asio::ip::tcp::socket>(ios);
            acceptor.accept(*p_socket);
            //std::thread(client_session, p_socket);
            PRINT_INFO("new client connect: {}", p_socket->remote_endpoint().address().to_string());
            std::thread *p_thread = new std::thread(client_session, p_socket);
            p_thread->detach();
        }
    }


    void service_module_main(int argc, char **argv)
    {
        using namespace bz_robot;
        PRINT_INFO("init");

        thread_rpc::Client task_client(MACRO_STR(MSG_ID_SERVER_TASK));
        task_client.call("import_config", std::string(argv[1]));
        task_client.call("task_module_start");


        std::vector<std::thread*> thread_list;
        //thread_list.push_back(new std::thread(bz_robot::on_recv_redis_msg));
    //    thread_list.push_back(new std::thread(bz_robot::listening));
    //    for(auto it = thread_list.begin(); it != thread_list.end(); ++it)
    //    {
    //        (*it)->detach();
    //    }
        PRINT_INFO("server start");
        serivce_running();
    }

}


