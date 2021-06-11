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
#include "task/task_manager.h"
#include "common/back_trace.h"

namespace bz_robot
{
    static std::shared_ptr<ServiceManager> P_SERVICE_MANAGER;
    static std::shared_ptr<ServiceBase> P_SERVICE_CONNECT = std::make_shared<ServiceConnect>("UmConnect", "none", "");
    static std::shared_ptr<ServiceBase> P_SERVICE_DOCK = std::make_shared<ServiceDock>("UmDock", "none", "");
    static std::shared_ptr<ServiceBase> P_SERVICE_DRIVE = std::make_shared<ServiceDrive>("UmDrive", "none", "");
    static std::shared_ptr<ServiceBase> P_SERVICE_GOTO = std::make_shared<ServiceGoto>("UmGoto", "none", "");
    static std::shared_ptr<ServiceBase> P_SERVICE_GET_MAP = std::make_shared<ServiceGetMap>("UmGetMap", "none", "Type,Objs,Lines,Points,Resolution,NumPoints");
    static std::shared_ptr<ServiceBase> P_SERVICE_GET_MAP_NAME = std::make_shared<ServiceGetMapName>("UmGetMapName", "none", "name");
    static std::shared_ptr<ServiceBase> P_SERVICE_GET_NAME = std::make_shared<ServiceGetName>("UmGetName", "none", "name");

    static std::shared_ptr<ServiceBase> P_SERVICE_RECONFIG = std::make_shared<ServiceReconfig>("BZRobotReconfig", "none", "");
    static std::mutex MTX;
    static boost::asio::ip::tcp::socket* P_LAST_SOCKET = nullptr;

    static void init()
    {
        //test
        P_SERVICE_MANAGER = std::make_shared<ServiceManager>();
        P_SERVICE_MANAGER->register_service(P_SERVICE_CONNECT);
        P_SERVICE_MANAGER->register_service(P_SERVICE_DOCK);
        P_SERVICE_MANAGER->register_service(P_SERVICE_DRIVE);
        P_SERVICE_MANAGER->register_service(P_SERVICE_GOTO);
        P_SERVICE_MANAGER->register_service(P_SERVICE_GET_MAP);
        P_SERVICE_MANAGER->register_service(P_SERVICE_GET_MAP_NAME);
        P_SERVICE_MANAGER->register_service(P_SERVICE_GET_NAME);
        P_SERVICE_MANAGER->register_service(P_SERVICE_RECONFIG);
    }

    static std::string deal_msg(const std::string& data)
    {
        nlohmann::json j = nlohmann::json::parse(data);
        P_SERVICE_MANAGER->set_next_service(j["#CMD#"], data);
        std::string str_result;
//        if(P_SERVICE_MANAGER->run_services(j["#CMD#"], data, &str_result))
//        {
//            const size_t len = str_result.size();
//            str_result.insert(0, "$#");
//            str_result.insert(2, std::to_string(len).c_str());
//            str_result.insert(2+std::to_string(len).size(), "##");
//            str_result.insert(str_result.size(), "$~");
//        }
        return str_result;
    }

    static void accept_handler(const boost::system::error_code &ec)
    {
        PRINT_DEBUG("new client connected");
        if (!ec)
        {
            PRINT_INFO("address: {}", P_LAST_SOCKET->remote_endpoint().address().to_string());
            //boost::asio::async_write(sock, boost::asio::buffer(data), write_handler);
        }
    }

    void serivce_running()
    {
        // asio程序必须的io_service对象
        boost::asio::io_service ios;
        // 具体的服务器地址与端口
        boost::asio::ip::tcp::endpoint endpotion(boost::asio::ip::tcp::v4(), 7273);
        // 创建acceptor对象，当前的IPV4作为服务器地址(127.0.0.1 || 0.0.0.0)，接受端口7273的消息.
        boost::asio::ip::tcp::acceptor acceptor(ios, endpotion);
        // 打印当前服务器地址
        std::cout << "addr: " << acceptor.local_endpoint().address() << std::endl;
        // 打印当前服务器端口
        std::cout << "port: " << acceptor.local_endpoint().port() << std::endl;

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
        // 一个临时的socket对象
        boost::asio::ip::tcp::socket socket(ios);
        // 阻塞等待客户端连接，连接成功后返回socket, accept这个函数使用引用来调取socket.
        acceptor.accept(socket);
        // 打印与本机服务器取得连接的客户端IP地址
        std::cout << "client: " << socket.remote_endpoint().address() << std::endl;
        // 循环执行服务
        while (true)
        {
            try
            {
                // // 阻塞发送作者名称到客户端
                // socket.write_some(boost::asio::buffer("hello CSND_Ayo"));
                // 阻塞接收客户端发来的数据
                P_SERVICE_MANAGER->set_client_socket(&socket);
                size_t msg_size = 0;
                {
                    char buf_data[1024];
                    std::lock_guard<std::recursive_mutex> lock(* (P_SERVICE_MANAGER->asio_mtx()));
                    boost::system::error_code ec;
                    msg_size = socket.read_some(boost::asio::buffer(buf_data, 1024), ec);
                    msg = std::string(buf_data);
                }
                //std::lock_guard<std::recursive_mutex> lock(* (P_SERVICE_MANAGER->asio_mtx()));
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
                                //PRINT_DEBUG("recv data:\n{}\n\n", str_data.c_str());
                                bz_robot::deal_msg(str_data);
                                //std::string str_ret = std::move(bz_robot::deal_msg(str_data));
                                //socket.write_some(boost::asio::buffer(str_ret));
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
            catch (...)
            {
                PRINT_DEBUG("error occured");
                PRINT_DEBUG("waiting for new client ...");
                socket = boost::asio::ip::tcp::socket(ios);
                acceptor.accept(socket);
                // 打印与本机服务器取得连接的客户端IP地址
                PRINT_INFO("connect ({}) connected", socket.remote_endpoint().address().to_string());
                //std::cout << "client: " << socket.remote_endpoint().address() << std::endl;
            }

        }
    }
}


int main(int argc, char **argv)
{
    using namespace bz_robot;
    init_log("service_server");
    init();

    P_SERVICE_MANAGER->task_manager()->import_configs(argv[1]);
    P_SERVICE_MANAGER->task_manager()->start();
    std::vector<std::thread*> thread_list;
    //thread_list.push_back(new std::thread(bz_robot::on_recv_redis_msg));
//    thread_list.push_back(new std::thread(bz_robot::listening));
//    for(auto it = thread_list.begin(); it != thread_list.end(); ++it)
//    {
//        (*it)->detach();
//    }

    std::cout << "server start." << std::endl;
    serivce_running();
    std::cout << "server end." << std::endl;
    return 0;
}


#if 0

//服务端
//#include "stdafx.h"
#include <iostream>
#include <string.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

using boost::asio::ip::tcp;
using boost::asio::ip::address;

class session : public boost::enable_shared_from_this<session> {
public:
    session(boost::asio::io_service &io_service) : socket_(io_service)
    {
    }

    void start() {
        static tcp::no_delay option(true);
        socket_.set_option(option);

        boost::asio::async_read(socket_,
                                      sbuf_,
                                      boost::bind(&session::handle_read,
                                                  shared_from_this(),
                                                  boost::asio::placeholders::error,
                                                  boost::asio::placeholders::bytes_transferred));
    }

    tcp::socket &socket() {
        return socket_;
    }

private:
    void handle_write(const boost::system::error_code& error, size_t bytes_transferred) {
        boost::asio::async_read(socket_,
                                      sbuf_,
                                      boost::bind(&session::handle_read,
                                                  shared_from_this(),
                                                  boost::asio::placeholders::error,
                                                  boost::asio::placeholders::bytes_transferred));
    }

    void handle_read(const boost::system::error_code& error, size_t bytes_transferred) {
//        std::string str;
//        boost::asio::buffer buf(str, 10000);
//        sbuf_ >> buf;
        //boost::asio::streambuf myBuffer;
        std::string myString;

        // Convert streambuf to std::string
        std::istream(&sbuf_) >> myString;

        std::cout << "msg from client: " <<  socket_.remote_endpoint().address().to_string() << std::endl;
        std::cout << myString << std::endl;
    }

private:
    tcp::socket socket_;
    boost::asio::streambuf sbuf_;
};

typedef boost::shared_ptr<session> session_ptr;

class server {
public:
    server(boost::asio::io_service &io_service, tcp::endpoint &endpoint)
        : io_service_(io_service), acceptor_(io_service, endpoint)
    {
        session_ptr new_session(new session(io_service_));
        acceptor_.async_accept(new_session->socket(),
                               boost::bind(&server::handle_accept,
                                           this,
                                           new_session,
                                           boost::asio::placeholders::error));
    }

    void handle_accept(session_ptr new_session, const boost::system::error_code& error) {
        if (error) {
            return;
        }
        std::cout << "new client: " <<  new_session->socket().remote_endpoint().address().to_string() << std::endl;
        new_session->start();
        new_session.reset(new session(io_service_));
        acceptor_.async_accept(new_session->socket(), boost::bind(&server::handle_accept, this, new_session,
                                                                  boost::asio::placeholders::error));
    }

    void run() {
        io_service_.run();
    }

private:
    boost::asio::io_service &io_service_;
    tcp::acceptor acceptor_;
};

int main(int argc, char* argv[])
{
    boost::asio::io_service io_service;
    tcp::endpoint endpoint(tcp::v4(), 7273);

    server s(io_service, endpoint);
    s.run();
    std::cout << "exit\n";
    return 0;
}

#endif

