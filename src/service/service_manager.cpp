#include "service/service_manager.h"
#include "service/service_base.h"
#include <stdio.h>
#include <chrono>
#include <list>
#include <unordered_map>
#include <semaphore.h>
#include "common/data_types.h"
#include "common/print.h"
#include "common/periodic_task.h"
#include "common/thread_rpc.h"
#include "common/common.h"
#include "common/msg_id.h"
#include "task/task_manager.h"


namespace bz_robot
{

class ServiceItem
{
public:
    ServiceItem():p_service(nullptr) {}
    ServiceBase* p_service = nullptr;
    std::string service_argc;
};

class ServiceManagerParams
{
public:
    ServiceItem cur_service;
    std::list<ServiceItem> p_service_list;
    std::map<std::string, ServiceBase*> p_service_map;
    std::mutex mtx;
    std::vector<std::thread*> thread_list;
    //TaskManager *p_task_manager;
    //std::shared_ptr<thread_rpc::Client> p_task_manager_client;
    // p_service - cycle_time
    std::unordered_map<ServiceBase*, uint32_t> background_service_map;
    struct event_base* p_event_base = nullptr;
    std::atomic<bool> flag_is_exit_service_list_monitor;
    //std::atomic<bool> flag_is_exit_running_background_service;
    std::atomic<bool> flag_is_exit_running_service;
    std::atomic<bool> flag_is_running_service_list_monitor;
    std::atomic<bool> flag_is_running_running_service;

    sem_t sem_awake_monitor;
    sem_t sem_awake_worker;
};

typedef struct {
    ServiceManager *p_manager;
    ServiceBase *p_service;
}ServiceBackgroundEventParam;

static void event_background_empty_service(int fd, short events, void *p_event_params);
static void event_background_service(int fd, short events, void * p_event_params);

ServiceManager::ServiceManager()
{
    mp_params = std::make_shared<ServiceManagerParams>();
    mp_params->flag_is_exit_service_list_monitor = false;
    //mp_params->flag_is_exit_running_background_service = false;
    mp_params->flag_is_exit_running_service = false;
    mp_params->flag_is_running_service_list_monitor = true;
    mp_params->flag_is_running_running_service = true;
    if(sem_init(&(mp_params->sem_awake_monitor), 0, 0) == -1)    //初始化信号量
    {
        PRINT_ERROR("error: sem_init failed!\r\n");
        exit(-1);
    }
    if(sem_init(&(mp_params->sem_awake_worker), 0, 0) == -1)    //初始化信号量
    {
        PRINT_ERROR("error: sem_init failed!\r\n");
        exit(-1);
    }

    mp_params->thread_list.push_back(new std::thread(&ServiceManager::service_list_monitor, this));
    mp_params->thread_list.push_back(new std::thread(&ServiceManager::running_service, this));
    for(auto it = mp_params->thread_list.begin(); it != mp_params->thread_list.end(); ++it)
    {
        (*it)->detach();
    }
    mp_socket.reset();
}

ServiceManager::~ServiceManager()
{

    if(sem_destroy( &(mp_params->sem_awake_monitor)) != 0 )
    {
        PRINT_ERROR("error on destroy sem, exit \n");
        exit(-1);
    }
    if(sem_destroy( &(mp_params->sem_awake_worker)) != 0 )
    {
        PRINT_ERROR("error on destroy sem, exit \n");
        exit(-1);
    }
}

bool ServiceManager::stop()
{
    mp_params->flag_is_exit_service_list_monitor = true;
    mp_params->flag_is_exit_running_service = true;

    while (true)
    {
        if(!mp_params->flag_is_running_service_list_monitor &&
                !mp_params->flag_is_running_running_service)
        {
            break;
        }
        else
        {
            std::this_thread::yield();
        }
    }
    for(auto it = mp_params->thread_list.begin(); it != mp_params->thread_list.end(); ++it)
    {
        delete (*it);
    }

    return true;
}

inline const ServiceManagerParams * const ServiceManager::params() const
{
    return mp_params.get();
}

bool ServiceManager::register_service(std::shared_ptr<ServiceBase> p_service)
{
    if(!p_service)
    {
        return false;
    }
    const std::string& cmd = p_service->cmd();
    if(mp_params->p_service_map.find(cmd) == mp_params->p_service_map.end())
    {
        p_service->set_service_manager(this);
        mp_params->p_service_map[cmd] = p_service.get();
        return true;
    }
    return false;
}

bool ServiceManager::set_next_service(const std::string &cmd, const std::string &argc)
{
    if(mp_params->p_service_map.find(cmd) == mp_params->p_service_map.end())
    {
        PRINT_ERROR("unsupported service: {}", cmd);
        return false;
    }

    ServiceItem service_item;
    service_item.p_service = mp_params->p_service_map[cmd];
    service_item.service_argc = argc;
    PRINT_DEBUG("add service: {}", service_item.p_service->cmd());

    std::lock_guard<std::mutex> temp_lock(mp_params->mtx);
    mp_params->p_service_list.push_back(service_item);

    if(sem_post( &(mp_params->sem_awake_monitor)) != 0 )
    {
        PRINT_ERROR("error on pose sem, exit \n");
        exit(-1);
    }
    return true;
}

void ServiceManager::service_list_monitor()
{
    while (true)
    {
        //PERIODIC_MS_TASK(20);

        if(mp_params->flag_is_exit_service_list_monitor)
        {
            PRINT_INFO("exit service list monitor");
            mp_params->flag_is_running_service_list_monitor = true;
            return;
        }

        //ServiceItem *p_cur_service_item = nullptr;
        std::unique_lock<std::mutex> temp_lock(mp_params->mtx);
        //PRINT_ERROR("running monitor, mp_params->p_service_list.size = {}", mp_params->p_service_list.size());
        if(!mp_params->p_service_list.empty())
        {
            if(mp_params->cur_service.p_service != nullptr)
            {
                if(mp_params->cur_service.p_service->is_locked())
                {
                    //nothing to do, wait cur task execute finish.
                }
                else
                {
                    if(mp_params->cur_service.p_service->stop())
                    {
                        mp_params->cur_service = mp_params->p_service_list.front();
                        mp_params->p_service_list.erase(mp_params->p_service_list.begin());
                    }
                    else
                    {
                        PRINT_ERROR("cur service can't stop, {}", mp_params->cur_service.p_service->cmd());
                    }
                }
            }
            else
            {
                mp_params->cur_service = mp_params->p_service_list.front();
                mp_params->p_service_list.erase(mp_params->p_service_list.begin());
                //mp_params->p_service_list.clear();

            }

            if(mp_params->cur_service.p_service != nullptr)
            {
                PRINT_INFO("seng cmd {} to worker", mp_params->cur_service.p_service->cmd());
                printf("mp_params->cur_service.p_service = %p\n", mp_params->cur_service.p_service);
                if(sem_post( &(mp_params->sem_awake_worker)) != 0 )
                {
                    PRINT_ERROR("error on pose sem, exit \n");
                    exit(-1);
                }
            }
        }
        else
        {
//            temp_lock.unlock();
//            if(sem_wait( &(mp_params->sem_awake_monitor)) != 0 )
//            {
//                PRINT_ERROR("error on wait sem");
//            }
        }

    }
}


bool ServiceManager::running_service()
{
    while (true)
    {
        //PERIODIC_MS_TASK(10);
        if(sem_wait( &(mp_params->sem_awake_worker)) != 0 )
        {
            PRINT_ERROR("error on wait sem");
        }
        PRINT_INFO("worker awake");
        if(mp_params->flag_is_exit_running_service)
        {
            mp_params->flag_is_running_running_service = true;
            PRINT_INFO("exit background service");
            break;
        }
        //TaskBase *p_task = nullptr;
        ServiceItem cur_service_item;
        {
            std::lock_guard<std::mutex> temp_lock(mp_params->mtx);
            printf("mp_params->cur_service.p_service = %p\n", mp_params->cur_service.p_service);
            cur_service_item = mp_params->cur_service;
            mp_params->cur_service.p_service = nullptr;
        }

        if(cur_service_item.p_service != nullptr)
        {
            PRINT_INFO("worker service: {}", cur_service_item.p_service->cmd());
            //cur_service_item.p_service->set_cmd(cur_service_item.service_argc);
            //PRINT_DEBUG("cur argc: {}", cur_service_item.service_argc);
            std::string ret_str;
            PRINT_DEBUG("running serivice: {}, {}\n", cur_service_item.p_service->cmd(), cur_service_item.service_argc);
            cur_service_item.p_service->set_msg(cur_service_item.service_argc);
            if(!cur_service_item.p_service->run(&ret_str))
            {
                PRINT_ERROR("running service: {} error", cur_service_item.p_service->cmd());
            }
        }
        else
        {
            PRINT_DEBUG("no task running");
            // switch to idel task
        }

    }
    return true;
}



std::vector<const ServiceBase*> ServiceManager::total_service()
{
    std::vector<const ServiceBase*> service_list;
    uint32_t i = 0;
    for(auto it = mp_params->p_service_map.begin(); it != mp_params->p_service_map.end(); ++it)
    {
        service_list.push_back(it->second);
    }
    return service_list;
}


bool ServiceManager::set_client_socket(std::shared_ptr<boost::asio::ip::tcp::socket> p_socket)
{
    mp_socket = p_socket;
}

bool ServiceManager::send_msg_to_client(std::string msg)
{
    //PRINT_DEBUG("try to get lock");
    std::lock_guard<std::recursive_mutex> lock(m_asio_mtx);
    //PRINT_INFO("---------------------begin to send msg------------------------");
    if(mp_socket)
    {

        const size_t len = msg.size();
        msg.insert(0, "$#");
        msg.insert(2, std::to_string(len).c_str());
        msg.insert(2+std::to_string(len).size(), "##");
        msg.insert(msg.size(), "$~");
        try
        {
            mp_socket->write_some(boost::asio::buffer(msg));
            mp_socket->write_some(boost::asio::buffer(std::string("")));
            //PRINT_DEBUG("write back\n");
        }
        catch(...)
        {
            PRINT_DEBUG("write error");
            mp_socket.reset();
        }
    }
    //PRINT_INFO("---------------------end of send msg------------------------\n");
}

}
