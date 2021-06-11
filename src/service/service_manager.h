#ifndef __SERVICE_MANAGER__
#define __SERVICE_MANAGER__
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <boost/asio.hpp>

namespace bz_robot
{
class ServiceBase;
class TaskManager;
class ServiceManagerParams;

class ServiceManager
{
public:
    ServiceManager();
    ~ServiceManager();
    bool stop();
    inline const ServiceManagerParams * const params() const;
    bool register_service(std::shared_ptr<ServiceBase> p_service);
    bool set_background_service(ServiceBase* p_service, int time_interval_ms);
    bool remove_background_service(ServiceBase *p_service);
    bool set_next_service(const std::string &cmd, const std::string& argc);
    bool running_service();
    //bool running_background_service();
    //bool run_services(const std::string &cmd, const std::string& argc, std::string *p_result);
    std::vector<const ServiceBase *> total_service();
//    //TaskManager * const task_manager() const;
//    inline void lock_asio();
//    inline void unlock_asio();
//    std::recursive_mutex* asio_mtx();
    bool set_client_socket(std::shared_ptr<boost::asio::ip::tcp::socket> p_socket);
    bool send_msg_to_client(std::string msg);

private:
    std::shared_ptr<ServiceManagerParams> mp_params;
    std::shared_ptr<boost::asio::ip::tcp::socket> mp_socket;
    std::recursive_mutex m_asio_mtx;
private:
    //void add_new_background_service(ServiceBase* p_service, int time_interval_ms);
    void service_list_monitor();
};
}
#endif //__SERVICE_MANAGER__
