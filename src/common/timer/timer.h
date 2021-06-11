#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <stdint.h>
#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <queue>
#include <semaphore.h>

namespace bz_robot
{
namespace timer
{
class noncopyable
{
protected:
    noncopyable() = default;
    ~noncopyable() = default;
private:
    noncopyable(const noncopyable&) = delete;
    const noncopyable& operator=( const noncopyable& ) = delete;
};

class Timer;

class TimerStamp
{
public:
    //时间轮时刻
    uint8_t timer_slot_index;
    //时间轮外圈时刻，即时间轮转完一圈，外圈时刻+1
    uint32_t timer_cycle_index;
};

class TimerTaskItem
{
public:
    TimerTaskItem() : p_timer(nullptr) {}
    //~TimerTaskItem(){printf("~TimerTaskItem\n");}

    std::atomic<bool> is_running;
    std::atomic<bool> is_alive;
    TimerStamp timer_stamp;
    //表示被延误了多少个slot和cycle
    TimerStamp delayed_timer_stamp;
    std::function<void(TimerTaskItem *, void*)> func;
    void* p_arg_list;
    Timer* p_timer;
    std::recursive_mutex mtx;
    std::string name;
public:
    void add_task(const uint32_t& cycle_time_ms, void* p_args);
    void remove_task();
};

class TimerSlotItem : public noncopyable
{
public:
    TimerSlotItem()
    {
        p_mtx = new std::recursive_mutex();
    }
private:
    struct cmp
    {
        //优先延迟任务
        bool operator() (const TimerTaskItem* a, const TimerTaskItem* b)
        {
            if(a == nullptr || b == nullptr)
            {
                return true;
            }

            bool result = true;
            if(a->delayed_timer_stamp.timer_cycle_index == b->delayed_timer_stamp.timer_cycle_index)
            {
                if(a->delayed_timer_stamp.timer_slot_index == b->delayed_timer_stamp.timer_slot_index)
                {
                    if(a->timer_stamp.timer_cycle_index == b->timer_stamp.timer_cycle_index)
                    {
                        result = a->timer_stamp.timer_slot_index > b->timer_stamp.timer_slot_index;
                    }
                    else
                    {
                        result = a->timer_stamp.timer_cycle_index > b->timer_stamp.timer_cycle_index;
                    }
                }
                else
                {
                    result = a->delayed_timer_stamp.timer_slot_index > b->delayed_timer_stamp.timer_slot_index;
                }
            }
            else
            {
                result = a->delayed_timer_stamp.timer_cycle_index > b->delayed_timer_stamp.timer_cycle_index;
            }
            return result;
        };
    };
public:
    std::recursive_mutex *p_mtx;
    std::priority_queue<TimerTaskItem*, std::vector<TimerTaskItem*>, cmp> p_task_list;
};

class TimerWorkThreadParams
{
public:
    std::atomic<bool> is_thread_running;
    std::thread* p_thread;
    sem_t sem;
    TimerTaskItem* p_task;
};


class TimerParams
{
public:
    uint8_t timer_wheel_num;
    uint32_t timer_wheel_step_ms;
    uint8_t timer_work_thread_num;
    TimerStamp cur_timer_stamp;
    std::mutex mtx_timer;
    std::vector<TimerSlotItem*> timer_wheel_slots;
    std::vector<TimerWorkThreadParams*> timer_work_thread_task;
};

class Timer
{

public:
    Timer(uint32_t timer_wheel_step_ms, uint8_t timer_wheel_num, uint8_t work_thread_num);

    ~Timer() {}

    template<typename Function>
    void add_task(const std::string& name, const uint32_t& cycle_time_ms, Function f, void* p_args)
    {
        //printf("this addr = %p\n", this);
        TimerTaskItem* p_task = new TimerTaskItem();

        const uint32_t timer_wheel_index = cycle_time_ms / (mp_timer_params->timer_wheel_step_ms);
        mp_timer_params->mtx_timer.lock();
        const uint32_t index = mp_timer_params->cur_timer_stamp.timer_slot_index + timer_wheel_index;
        const uint32_t cycle_index = index / mp_timer_params->timer_wheel_num + mp_timer_params->cur_timer_stamp.timer_cycle_index;
        const uint32_t slot_index = index % mp_timer_params->timer_wheel_num;
        mp_timer_params->mtx_timer.unlock();
        p_task->p_timer = this;
        //printf("p_task->p_timer addr = %p\n", p_task->p_timer);
        p_task->timer_stamp.timer_slot_index = slot_index;
        p_task->timer_stamp.timer_cycle_index = cycle_index;
        p_task->is_running = false;
        p_task->is_alive = true;
        p_task->name = name;
        register_nonmember_func(p_task, std::move(f));
        p_task->p_arg_list = p_args;
        TimerSlotItem* p_timer_slot_item = mp_timer_params->timer_wheel_slots[slot_index];
        p_timer_slot_item->p_mtx->lock();
        p_timer_slot_item->p_task_list.push(p_task);
        p_timer_slot_item->p_mtx->unlock();
    }

    template<typename Function, typename Self, typename... Args>
    void add_task(const std::string& name, const uint32_t& cycle_time_ms, Function f, Self* self, void* p_args)
    {
        //printf("this addr = %p\n", this);
        //std::shared_ptr<TimerTaskItem> p_task = std::make_shared<TimerTaskItem>();
        TimerTaskItem *p_task = new TimerTaskItem();
        const uint32_t timer_wheel_index = cycle_time_ms / (mp_timer_params->timer_wheel_step_ms);
        mp_timer_params->mtx_timer.lock();
        p_task->p_timer = this;
        //printf("p_task->p_timer addr = %p\n", p_task->p_timer);
        const uint32_t index = mp_timer_params->cur_timer_stamp.timer_slot_index + timer_wheel_index;
        const uint32_t cycle_index = index / mp_timer_params->timer_wheel_num + mp_timer_params->cur_timer_stamp.timer_cycle_index;
        const uint32_t slot_index = index % mp_timer_params->timer_wheel_num;
        mp_timer_params->mtx_timer.unlock();

        p_task->timer_stamp.timer_slot_index = slot_index;
        p_task->timer_stamp.timer_cycle_index = cycle_index;
        p_task->is_running = false;
        p_task->is_alive = true;
        p_task->name = name;
        register_member_func(p_task, f, self);
        p_task->p_arg_list = p_args;
        TimerSlotItem* p_timer_slot_item = mp_timer_params->timer_wheel_slots[slot_index];
        p_timer_slot_item->p_mtx->lock();
        p_timer_slot_item->p_task_list.push(p_task);

        p_timer_slot_item->p_mtx->unlock();
    }
#if 0
    template<typename Function>
    void add_task(const uint32_t& cycle_time_ms, Function f, void* p_args)
    {
        //printf("this addr = %p\n", this);
        TimerTaskItem* p_task = new TimerTaskItem();

        const uint32_t timer_wheel_index = cycle_time_ms / (mp_timer_params->timer_wheel_step_ms);
        mp_timer_params->mtx_timer.lock();
        const uint32_t index = mp_timer_params->cur_timer_stamp.timer_slot_index + timer_wheel_index;
        const uint32_t cycle_index = index / mp_timer_params->timer_wheel_num + mp_timer_params->cur_timer_stamp.timer_cycle_index;
        const uint32_t slot_index = index % mp_timer_params->timer_wheel_num;
        mp_timer_params->mtx_timer.unlock();
        p_task->p_timer = this;
        //printf("p_task->p_timer addr = %p\n", p_task->p_timer);
        p_task->timer_stamp.timer_slot_index = slot_index;
        p_task->timer_stamp.timer_cycle_index = cycle_index;
        p_task->is_running = false;
        p_task->is_alive = true;
        register_nonmember_func(p_task, std::move(f));
        p_task->p_arg_list = p_args;
        TimerSlotItem* p_timer_slot_item = mp_timer_params->timer_wheel_slots[slot_index];
        p_timer_slot_item->p_mtx->lock();
        p_timer_slot_item->p_task_list.push(p_task);
        p_timer_slot_item->p_mtx->unlock();
    }

    template<typename Function, typename Self, typename... Args>
    void add_task(const uint32_t& cycle_time_ms, Function f, Self* self, void* p_args)
    {
        //printf("this addr = %p\n", this);
        //std::shared_ptr<TimerTaskItem> p_task = std::make_shared<TimerTaskItem>();
        TimerTaskItem *p_task = new TimerTaskItem();
        const uint32_t timer_wheel_index = cycle_time_ms / (mp_timer_params->timer_wheel_step_ms);
        mp_timer_params->mtx_timer.lock();
        p_task->p_timer = this;
        //printf("p_task->p_timer addr = %p\n", p_task->p_timer);
        const uint32_t index = mp_timer_params->cur_timer_stamp.timer_slot_index + timer_wheel_index;
        const uint32_t cycle_index = index / mp_timer_params->timer_wheel_num + mp_timer_params->cur_timer_stamp.timer_cycle_index;
        const uint32_t slot_index = index % mp_timer_params->timer_wheel_num;
        mp_timer_params->mtx_timer.unlock();

        p_task->timer_stamp.timer_slot_index = slot_index;
        p_task->timer_stamp.timer_cycle_index = cycle_index;
        p_task->is_running = false;
        p_task->is_alive = true;
        register_member_func(p_task, f, self);
        p_task->p_arg_list = p_args;
        TimerSlotItem* p_timer_slot_item = mp_timer_params->timer_wheel_slots[slot_index];
        p_timer_slot_item->p_mtx->lock();
        p_timer_slot_item->p_task_list.push(p_task);

        p_timer_slot_item->p_mtx->unlock();
    }
#endif
    void add_task(TimerTaskItem *p_task, const uint32_t& cycle_time_ms, void* p_args);
    bool remove_task(TimerTaskItem *p_task);
    void run();
    inline bool is_task_need_to_execute(const TimerStamp& cur_time_stamp, const TimerTaskItem *p_task_item);
    void thread_work(const uint32_t index);
private:
    std::shared_ptr<TimerParams> mp_timer_params;

    template<typename Function>
    struct invoker
    {
        static inline void apply_func(const Function& func, TimerTaskItem* p_task,
                                      void* p_args)
        {
            try
            {
                func(p_task, p_args);
            }
            catch(...)
            {

            }
        }

        template<typename Self>
        static inline void apply_member_func(const Function& func, Self* self, TimerTaskItem* p_task,
                                             void* p_args)
        {
            try
            {
                (*self.*func)(p_task, p_args);
            }
            catch(...)
            {

            }
        }
    };

    template<typename Function>
    void register_nonmember_func(TimerTaskItem* p_task, Function f)
    {
        p_task->func = {std::bind(&invoker<Function>::template apply_func, std::move(f), std::placeholders::_1,
                                 std::placeholders::_2)};
    }

    template<typename Function, typename Self>
    void register_member_func(TimerTaskItem* p_task, const Function& f, Self* self)
    {
        p_task->func = {std::bind(&invoker<Function>::template apply_member_func<Self>,
                                               f, self, std::placeholders::_1, std::placeholders::_2)};
    }
};

class GetTimer : public noncopyable
{
public:
  static Timer& timer()
  {
    static Timer intance(10, 10, 6);
    return intance;
  }
};

}
}

#endif // TIMER_H
