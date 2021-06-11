#include "timer.h"
#include "common/periodic_task.h"
#include "common/common.h"
#include "common/time_keeper.h"


namespace bz_robot
{

namespace timer
{
void TimerTaskItem::add_task(const uint32_t& cycle_time_ms, void* p_args)
{
    //printf("p_task->p_timer addr = %p\n", p_timer);
    if(p_timer)
    {
        p_timer->add_task(this, cycle_time_ms, p_args);
    }
}

void TimerTaskItem::remove_task()
{
    if(p_timer)
    {
        p_timer->remove_task(this);
    }
}


Timer::Timer(uint32_t timer_wheel_step_ms, uint8_t timer_wheel_num, uint8_t work_thread_num)
{
    mp_timer_params = std::make_shared<TimerParams>();
    mp_timer_params->timer_wheel_step_ms = timer_wheel_step_ms;
    mp_timer_params->timer_wheel_num = timer_wheel_num;
    mp_timer_params->timer_work_thread_num = work_thread_num;

    mp_timer_params->timer_wheel_slots.resize(timer_wheel_num);
    for(int i = 0; i != timer_wheel_num; ++i )
    {
        mp_timer_params->timer_wheel_slots[i] = new TimerSlotItem();
    }

    mp_timer_params->timer_work_thread_task.resize(work_thread_num);
    for(int i = 0; i != work_thread_num; ++i )
    {
        mp_timer_params->timer_work_thread_task[i] = new TimerWorkThreadParams();
        mp_timer_params->timer_work_thread_task[i]->is_thread_running = false;
        mp_timer_params->timer_work_thread_task[i]->p_thread
                = new std::thread(&Timer::thread_work, this, i);
        //mp_timer_params->timer_work_p_thread_list.emplace_back(new std::thread());
        //mp_timer_params->timer_work_p_thread_list[i]->detach();
        if(sem_init(&(mp_timer_params->timer_work_thread_task[i]->sem), 0, 0) == -1)    //初始化信号量
        {
            printf("error: sem_init failed!\r\n");
            exit(-1);
        }
    }
}


void Timer::add_task(TimerTaskItem *p_task, const uint32_t& cycle_time_ms, void* p_args)
{
    //printf("this addr = %p\n", this);
    const uint32_t timer_wheel_index = cycle_time_ms / (mp_timer_params->timer_wheel_step_ms);
    mp_timer_params->mtx_timer.lock();
    const uint32_t index = mp_timer_params->cur_timer_stamp.timer_slot_index + timer_wheel_index;
    const uint32_t cycle_index = index / mp_timer_params->timer_wheel_num + mp_timer_params->cur_timer_stamp.timer_cycle_index;
    const uint32_t slot_index = index % mp_timer_params->timer_wheel_num;
    mp_timer_params->mtx_timer.unlock();
    p_task->is_alive = true;
    p_task->timer_stamp.timer_slot_index = slot_index;
    p_task->timer_stamp.timer_cycle_index = cycle_index;

    p_task->p_arg_list = p_args;
    TimerSlotItem* p_timer_slot_item = mp_timer_params->timer_wheel_slots[slot_index];
    p_timer_slot_item->p_mtx->lock();
    p_timer_slot_item->p_task_list.push(p_task);
    p_timer_slot_item->p_mtx->unlock();
}

bool Timer::remove_task(TimerTaskItem *p_task)
{
    //mp_timer_params->mtx_timer.lock();
    //delete *p_task;
    if(p_task)
    {
        p_task->mtx.lock();
        p_task->is_alive = false;
        p_task->mtx.unlock();
        return true;
    }
    return false;
    //mp_timer_params->mtx_timer.unlock();
}

void Timer::run()
{
    const uint32_t timer_wheel_num = mp_timer_params->timer_wheel_num;
    const uint32_t time_step_ms = mp_timer_params->timer_wheel_step_ms;
    while (true)
    {
        PERIODIC_MS_TASK(time_step_ms);
        //printf("timer stamp: %.3f\n", time_stamp_us() * 1e-6);
        ++mp_timer_params->cur_timer_stamp.timer_slot_index;
        mp_timer_params->cur_timer_stamp.timer_cycle_index += mp_timer_params->cur_timer_stamp.timer_slot_index / timer_wheel_num;
        mp_timer_params->cur_timer_stamp.timer_slot_index  = mp_timer_params->cur_timer_stamp.timer_slot_index % timer_wheel_num;
        const uint32_t timer_slot_index = mp_timer_params->cur_timer_stamp.timer_slot_index;
        TimerSlotItem* p_timer_slot_item = mp_timer_params->timer_wheel_slots[timer_slot_index];
        //printf("cur timer stamp: %d - %d\n", timer_slot_index, mp_timer_params->cur_timer_stamp.timer_cycle_index);
        while(true)
        {
            std::lock_guard<std::recursive_mutex> lock(*(p_timer_slot_item->p_mtx));
            //printf("run in %d, size = %d\n", timer_slot_index, p_timer_slot_item->p_task_list.size());
            //p_timer_slot_item->p_mtx->lock();
            //auto & p_task_list = mp_timer_params->timer_wheel_slots[timer_slot_index]->p_task_list;
            auto & p_task_list = p_timer_slot_item->p_task_list;
            //p_timer_slot_item->p_mtx->unlock();
            if(p_task_list.empty())
            {
                //printf("task empty, break\n");
                break;
            }
            TimerTaskItem* task_item = p_task_list.top();
//            printf("top task delayed stamp: %d, %d, timer stamp: %d, %d\n",
//                   task_item->delayed_timer_stamp.timer_slot_index,
//                   task_item->delayed_timer_stamp.timer_cycle_index,
//                   task_item->timer_stamp.timer_slot_index,
//                   task_item->timer_stamp.timer_cycle_index);

            //if task has been delayed
            if(is_task_need_to_execute(mp_timer_params->cur_timer_stamp, task_item))
            {
                if(task_item->is_running)
                {
                    ++task_item->delayed_timer_stamp.timer_slot_index;
                    task_item->delayed_timer_stamp.timer_cycle_index += task_item->delayed_timer_stamp.timer_slot_index / timer_wheel_num;
                    task_item->delayed_timer_stamp.timer_slot_index  = task_item->delayed_timer_stamp.timer_slot_index % timer_wheel_num;

                    const uint32_t next_timer_slot_index = (timer_slot_index + 1) % timer_wheel_num;
                    mp_timer_params->timer_wheel_slots[next_timer_slot_index]->p_mtx->lock();
                    mp_timer_params->timer_wheel_slots[next_timer_slot_index]->p_task_list.push(task_item);
                    mp_timer_params->timer_wheel_slots[next_timer_slot_index]->p_mtx->unlock();
                    p_task_list.pop();
                    //no work thread empty
                    //move task to next time slot
                    //printf("task over time, move to next timer slot(%d)\n", next_timer_slot_index);
//                    TimerTaskItem* top_task_item = mp_timer_params->timer_wheel_slots[next_timer_slot_index]->p_task_list.top();
//                    printf("delayed stamp: %d, %d, timer stamp: %d, %d\n",
//                           top_task_item->delayed_timer_stamp.timer_slot_index,
//                           top_task_item->delayed_timer_stamp.timer_cycle_index,
//                           top_task_item->timer_stamp.timer_slot_index,
//                           top_task_item->timer_stamp.timer_cycle_index);
                    continue;
                }

                int free_work_thread_index = 0;
                for(free_work_thread_index = 0; free_work_thread_index < mp_timer_params->timer_work_thread_num; ++free_work_thread_index)
                {
                    if(!mp_timer_params->timer_work_thread_task[free_work_thread_index]->is_thread_running)
                    {
                        break;
                    }
                }
                //printf("free_work_thread_index = %d\n", free_work_thread_index);
                if(free_work_thread_index < mp_timer_params->timer_work_thread_num)
                {
                    task_item->delayed_timer_stamp.timer_slot_index = 0;
                    task_item->delayed_timer_stamp.timer_cycle_index = 0;
                    //send task to thread
                    mp_timer_params->timer_work_thread_task[free_work_thread_index]->p_task = task_item;
                    //printf("[%f] send task %d\n", time_stamp(), free_work_thread_index);
                    task_item->is_running = true;
                    mp_timer_params->timer_work_thread_task[free_work_thread_index]->is_thread_running = true;
                    if(sem_post( &(mp_timer_params->timer_work_thread_task[free_work_thread_index]->sem)) != 0 )
                    {
                        PRINT_ERROR("error on pose sem, exit \n");
                        exit(-1);
                    }

                    p_task_list.pop();
                    continue;
                }
                else
                {
                    ++task_item->delayed_timer_stamp.timer_slot_index;
                    task_item->delayed_timer_stamp.timer_cycle_index += task_item->delayed_timer_stamp.timer_slot_index / timer_wheel_num;
                    task_item->delayed_timer_stamp.timer_slot_index  = task_item->delayed_timer_stamp.timer_slot_index % timer_wheel_num;

                    const uint32_t next_timer_slot_index = (timer_slot_index + 1) % timer_wheel_num;
                    mp_timer_params->timer_wheel_slots[next_timer_slot_index]->p_mtx->lock();
                    mp_timer_params->timer_wheel_slots[next_timer_slot_index]->p_task_list.push(task_item);
                    mp_timer_params->timer_wheel_slots[next_timer_slot_index]->p_mtx->unlock();
                    p_task_list.pop();
                    //no work thread empty
                    //move task to next time slot
                    PRINT_WARN("no work thread empty, move [{}] to next timer slot{}\n", task_item->name, next_timer_slot_index);
                    continue;
                }
            }
            else
            {
                //printf("task no need to execute, break\n");
                break;
            }
        }
        //std::this_thread::sleep_for(std::chrono::milliseconds(time_step_ms));
    }
}

inline bool Timer::is_task_need_to_execute(const TimerStamp& cur_time_stamp, const TimerTaskItem *p_task_item)
{
//    if(p_task_item->is_running)
//    {
//        return false;
//    }
    if(p_task_item->delayed_timer_stamp.timer_slot_index != 0
            || p_task_item->delayed_timer_stamp.timer_cycle_index != 0)
    {
        return true;
    }
//        if(p_task_item->timer_stamp.timer_slot_index == cur_time_stamp.timer_slot_index
//                && p_task_item->timer_stamp.timer_cycle_index == cur_time_stamp.timer_cycle_index)
//        {
//            return true;
//        }

    // slot index 肯定是对的
    if(p_task_item->timer_stamp.timer_cycle_index == cur_time_stamp.timer_cycle_index)
    {
        return true;
    }
    return false;
}

void Timer::thread_work(const uint32_t index)
{
    while (true)
    {
        TimerTaskItem* p_task = nullptr;
        try
        {
            if(sem_wait(&(mp_timer_params->timer_work_thread_task[index]->sem)) != 0 )
            {
                PRINT_ERROR("error on wait sem, exit \n");
                exit(-1);
            }

            //printf("[%f] recv task, %d\n", time_stamp(), index);
            mp_timer_params->timer_work_thread_task[index]->is_thread_running = true;
            p_task = mp_timer_params->timer_work_thread_task[index]->p_task;
            //RECORD_TIME(p_task->name);
            if(p_task && p_task->is_alive)
            {
                p_task->is_running = true;
                //call task
                //printf("call task\n");
                p_task->is_alive = false;
                p_task->mtx.lock();
                p_task->func(p_task, p_task->p_arg_list);
                p_task->mtx.unlock();
                //p_task->is_running = false;
            }
        }
        catch(std::exception& e)
        {
            if(p_task)
            {
                p_task->mtx.unlock();
            }
            PRINT_ERROR("{} exception: {}\n", p_task->name, e.what());
        }
        catch (...)
        {
            if(p_task)
            {
                p_task->mtx.unlock();
            }
            PRINT_ERROR("{} unknown error occured\n", p_task->name);
        }

        if(p_task)
        {
            p_task->is_running = false;
            //需要额外加判断条件
            //不再继续执行才能销毁
            if(! p_task->is_alive)
            {
                delete p_task;
            }

        }
        mp_timer_params->timer_work_thread_task[index]->is_thread_running = false;

    }
    PRINT_ERROR("exit task: %d\n", index);
}
}
}
