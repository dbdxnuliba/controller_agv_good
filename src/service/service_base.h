#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <mutex>
#include "common/json.hpp"
#include "common/timer/timer.h"


namespace bz_robot
{
class ServiceManager;
class ServiceBase
{
public:
    ServiceBase(const std::string cmd, const std::string arg, const std::string ret):
        m_cmd(cmd),
        m_arg(arg),
        m_ret(ret),
        m_lock_flag(false),
        mp_task(nullptr),
        m_gap(-1){}
    virtual ~ServiceBase()
    {
        m_destroy_mtx.lock();
        if(mp_task)
        {
            mp_task->remove_task();
        }
        m_destroy_mtx.unlock();
    }
    virtual const std::string& cmd() const { return m_cmd;}
    virtual const std::string& arg() const { return m_arg;}
    virtual const std::string& ret() const { return m_ret;}
    virtual bool set_service_manager(ServiceManager* p_manager){mp_manager = p_manager;}
    virtual bool init(){return true;}
    virtual bool set_msg(const std::string& argc) {m_json_msg = argc;}

    virtual inline bool is_locked() {return m_lock_flag;}
    //virtual bool run(std::string* p_result){return true;}
    //virtual bool stop() {if(!m_lock_flag){return true;} return false;}
    virtual bool stop() {if(m_lock_flag){return false;} return true;}
    //virtual inline uint32_t time_interval_ms() const {return m_time_interval_ms;}
    virtual bool run(std::string* p_result)
    {
        p_result->clear();
        const std::string& argc = m_json_msg;
        nlohmann::json j = nlohmann::json::parse(argc);
        m_gap = int(j["#GAP#"]);

        if(mp_task == nullptr)
        {
            if(m_gap > 0)
            {
                timer::Timer& t = timer::GetTimer::timer();
                t.add_task(m_cmd, m_gap, &ServiceBase::execute, this, nullptr);
                //run timer task
            }
            else
            {
                //run task
                execute(nullptr, nullptr);
            }
        }
        else
        {
            if(m_gap > 0)
            {
                //nothing to do
            }
            else
            {
                mp_task->remove_task();
                mp_task = nullptr;
            }
        }
        return true;
    }

    virtual void execute(timer::TimerTaskItem* p_task, void *p_args)
    {
        std::lock_guard<std::mutex> destroy_lock(m_destroy_mtx);
        if(m_gap > 0)
        {
            mp_task = p_task;
            if(mp_task != nullptr)
            {
                p_task->add_task(m_gap, p_args);
            }
        }
        execute_once(p_args);
    }

    virtual void execute_once(void *p_args) = 0;
protected:
    virtual inline void lock() {m_lock_flag = true;}
    virtual inline void unlock() {m_lock_flag = false;}
protected:
    std::string m_cmd;
    std::string m_arg;
    std::string m_ret;
    std::string m_json_msg;
    ServiceManager* mp_manager;
    std::atomic<bool> m_lock_flag;
    //std::mutex m_mtx;
    std::mutex m_destroy_mtx;
    timer::TimerTaskItem* mp_task;
    int m_gap;
    //uint32_t m_time_interval_ms = -1;
};
}
