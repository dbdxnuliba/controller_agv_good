#include "task/task_navigation.h"
#include <mutex>
#include <thread>
#include <condition_variable>
//#include <event2/event.h>
//#include <event2/thread.h>
//#include "third_libs/rest_rpc/include/rest_rpc.hpp"
#include "common/msg_id.h"
#include "common/common.h"
#include "common/thread_rpc.h"
#include "common/periodic_task.h"
#include "common/print.h"
#include "common/prune.h"
#include "common/time_keeper.h"
#include "map/grid_map_interface.h"
#include "planner/frenet_optimal_planner/frenet_cartesian.h"


namespace bz_robot
{

static VectorX2<float> closest_point_on_line(const VectorX2<float> &cur_pose,
                                             const VectorX2<float> &a,
                                             const VectorX2<float> &b);
static float calculate_cte(const VectorX2<float> cur_point,
                           const VectorX2<float> point_from,
                           const VectorX2<float> point_to);


class GlobalPlannerParams
{
public:
    GlobalPlannerParams():
      global_path_lateral_error(0.5),
      global_planner_periodic_time_ms(2000),
      flag_is_global_planner_force_replan(false),
      flag_is_need_global_replan(false),
      flag_is_global_plan_success(false),
      flag_is_enable_global_path_smooth(true)
    {}
public:
    PathData last_global_path;
    Prune pruner_global_planner_global_path;
    FLOAT_T global_path_lateral_error; // unit m
    uint32_t global_planner_periodic_time_ms; // unit ms
    bool flag_is_new_global_path;

    bool flag_is_global_planner_force_replan;
    bool flag_is_need_global_replan;
    bool flag_is_global_plan_success;
    bool flag_is_enable_global_path_smooth;

    void reset()
    {
        last_global_path.clear();
        pruner_global_planner_global_path.reset();
        flag_is_global_plan_success = false;
        flag_is_new_global_path = false;
    }
};

class LocalPlannerParams
{
public:
    PathData last_local_path;
    Prune pruner_local_planner_global_path;
    Prune pruner_local_planner_local_path;
    Pose<FLOAT_T> last_local_planner_target;
    bool flag_is_local_planner_use_global_path = false;
    bool flag_is_local_planner_force_replan = false;
    FLOAT_T local_planner_replan_distance = 3.0; // unit m
    FLOAT_T local_path_lateral_error = 0.5; // unit m
    uint32_t local_planner_periodic_time_ms = 200;
    bool flag_is_local_plan_success = false;
    bool flag_is_enable_local_path_smooth = true;

    void reset()
    {
        last_local_path.clear();
        flag_is_local_plan_success = false;
        pruner_local_planner_global_path.reset();
        pruner_local_planner_local_path.reset();
    }
};

class TrackerParams
{
public:
    uint32_t tracker_periodic_time_ms = 30;// unit ms
};

class TaskNavigationParams
{
public:
    GlobalPlannerParams global_planner_params;
    LocalPlannerParams local_planner_params;
    TrackerParams tracker_params;

    std::shared_ptr<thread_rpc::Client> p_client_global_planner;
    std::shared_ptr<thread_rpc::Client> p_client_global_path_smoother;
    std::shared_ptr<thread_rpc::Client> p_client_local_planner;
    std::shared_ptr<thread_rpc::Client> p_client_local_path_smoother;
    std::shared_ptr<thread_rpc::Client> p_client_robot;
    std::shared_ptr<thread_rpc::Client> p_client_map;
    std::shared_ptr<thread_rpc::Client> p_client_odom;
    std::shared_ptr<thread_rpc::Client> p_client_tracker;

    std::mutex mtx_param;
    Msg<Pose<FLOAT_T>> msg_pose_goal;

    FLOAT_T distance_diff_tolerance = 0.3;
    FLOAT_T angle_diff_tolerance = 20;

    std::atomic<bool> flag_is_task_finished;
    std::atomic<bool> flag_is_new_task;
    std::atomic<bool> flag_is_stop_task;

    std::vector<std::thread> thread_pool;
    std::mutex mtx_condition;
    std::condition_variable condition;

    //std::promise<uint8_t> task_result;
    std::shared_ptr<std::promise<uint8_t>> p_task_result;
    std::mutex mtx_stop_run;
    void reset()
    {
        global_planner_params.reset();
        local_planner_params.reset();
        flag_is_new_task = false;
        flag_is_stop_task = false;
        flag_is_task_finished = false;
    }
};

TaskNavigation::TaskNavigation() : TaskBase()
{
    mp_params = new TaskNavigationParams();
    //mp_params = std::make_shared<TaskNavigationParams>();
    mp_params->p_client_robot = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_ROBOT));
    mp_params->p_client_map = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_MAP));
    mp_params->p_client_odom = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_LOCALIZATION));
    mp_params->p_client_global_planner = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_GLOBAL_PLAN));
    mp_params->p_client_local_planner = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_LOCAL_PLAN));
    mp_params->p_client_tracker = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_TRACKER));
    mp_params->p_client_global_path_smoother = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_GLOBAL_PATH_SMOOTH));
    mp_params->p_client_local_path_smoother = std::make_shared<thread_rpc::Client>(MACRO_STR(MSG_ID_SERVER_LOCAL_PATH_SMOOTH));

    mp_params->reset();
    mp_params->thread_pool.clear();
    mp_params->thread_pool.emplace_back(std::thread(&TaskNavigation::thread_global_plan, this));
    mp_params->thread_pool.emplace_back(std::thread(&TaskNavigation::thread_local_plan, this));
    mp_params->thread_pool.emplace_back(std::thread(&TaskNavigation::thread_tracker, this));

    for(int i = 0; i < mp_params->thread_pool.size(); ++i)
    {
        mp_params->thread_pool[i].detach();
    }
}

TaskNavigation::~TaskNavigation()
{

}

bool TaskNavigation::import_config(const char *file)
{
    RECORD_TIME();
    try
    {
        PRINT_INFO("read config: {}", file);
        std::ifstream i(file);
        if(i)
        {
            nlohmann::json j;
            i >> j;
            //global
            mp_params->global_planner_params.flag_is_global_planner_force_replan
                    = j["IS_GLOBAL_PLAN_FORCE_REPLAN"];
            mp_params->global_planner_params.global_path_lateral_error
                    = j["global_path_lateral_error"];
            mp_params->global_planner_params.global_planner_periodic_time_ms
                    = j["global_planner_periodic_time_ms"];
            mp_params->global_planner_params.flag_is_enable_global_path_smooth
                    = j["ENABLE_GLOBAL_SMOOTHER"];
            //local
            mp_params->local_planner_params.flag_is_local_planner_force_replan
                    = j["IS_LOCAL_PLAN_FORCE_REPLAN"];
            mp_params->local_planner_params.flag_is_local_planner_use_global_path
                    = j["IS_LOCAL_PLANNER_USE_GLOBAL_PATH"];
            mp_params->local_planner_params.local_planner_replan_distance
                    = j["LOCAL_PLANNER_REPLAN_DISTANCE"];
            mp_params->local_planner_params.local_path_lateral_error
                    = j["local_path_lateral_error"];
            mp_params->local_planner_params.local_planner_periodic_time_ms
                    = j["local_planner_periodic_time_ms"];
            mp_params->local_planner_params.flag_is_enable_local_path_smooth
                    = j["ENABLE_LOCAL_SMOOTHER"];
            //tracker
            mp_params->tracker_params.tracker_periodic_time_ms =
                    j["tracker_periodic_time_ms"];


            mp_params->distance_diff_tolerance
                    = j["distance_diff_tolerance"];
            mp_params->distance_diff_tolerance = fabs(mp_params->distance_diff_tolerance);
            mp_params->angle_diff_tolerance
                    = j["angle_diff_tolerance"];
            mp_params->angle_diff_tolerance = fabs(constraint_angle_d(radian_to_degree(mp_params->angle_diff_tolerance), -180, 180));
            return true;
        }
        else
        {
            PRINT_ERROR("can't read config files from: {}\n", file);
        }
    }
    catch(std::exception& e )
    {
        PRINT_ERROR("{}", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("un expexted occured");
    }
    std::ifstream i(file);
    if(i)
    {
        nlohmann::json j;
        i >> j;
        //std::cout << j << std::endl;
        PRINT_DEBUG("{}", j.dump(4));
    }

    return false;
}


bool TaskNavigation::init()
{
    return true;
}


//TaskNavigationParams* TaskNavigation::params()
//{
//    return mp_params;
//    //return mp_params;
//}

bool TaskNavigation::set_cmd(const std::string &cmd)
{

    nlohmann::json j = nlohmann::json::parse(cmd);

    mp_params->msg_pose_goal.time_stamp_us = std::move(time_stamp_us());
    //mm to m
    mp_params->msg_pose_goal.data.position.x = (FLOAT_T)j["poseX"] * 1e-3;
    mp_params->msg_pose_goal.data.position.y = (FLOAT_T)j["poseY"] * 1e-3;
    mp_params->msg_pose_goal.data.heading_angle = degree_to_radian((FLOAT_T)j["poseTh"]);

    mp_params->p_client_tracker->call("set_goal", mp_params->msg_pose_goal);

    PRINT_DEBUG("recv cmd:\n{}\n{}, {}, {}", cmd, mp_params->msg_pose_goal.data.position.x,
                mp_params->msg_pose_goal.data.position.y, mp_params->msg_pose_goal.data.heading_angle);
//    mp_params->p_client_global_planner->call("stop");
//    mp_params->p_client_local_planner->call("stop");
    return true;
}

RetMsg<std::string> TaskNavigation::run()
{
    PRINT_DEBUG("run");
    {
        //停止任务执行完后，才能下一次运行
        std::lock_guard<std::mutex> lock(mp_params->mtx_stop_run);
    }
    mp_params->p_task_result = std::make_shared<std::promise<uint8_t>>();
    //stop last plan
    //stop tracker
    // 理论上不需要，因为停止任务时会停止这些操作。这里是以防万一
    //mp_params->p_client_global_planner->call("stop");
    //mp_params->p_client_local_planner->call("stop");
    //-----------------------------------------------------

    mp_params->reset();
    mp_params->flag_is_new_task = true;
    mp_params->flag_is_task_finished = false;
    {
    std::unique_lock<std::mutex> server_lock(mp_params->mtx_condition);
    mp_params->condition.notify_all();
    }
    mp_params->p_task_result->get_future().get();
    {
        //停止任务执行完后，才能下一次运行
        std::lock_guard<std::mutex> lock(mp_params->mtx_stop_run);
    }
    PRINT_DEBUG("1111111111");
    mp_params->flag_is_new_task = false;
    mp_params->flag_is_task_finished = true;
    PRINT_DEBUG("222222222");
    mp_params->p_task_result.reset();
}

bool TaskNavigation::stop()
{
    PRINT_DEBUG("stop");
    //停止任务执行完后，才能下一次运行
    std::lock_guard<std::mutex> lock(mp_params->mtx_stop_run);

    mp_params->p_client_global_planner->call("stop");
    mp_params->p_client_local_planner->call("stop");
    mp_params->p_client_tracker->call("set_stop_signal");
    mp_params->p_client_robot->call("stop");



//    mp_params->last_global_path.clear();
//    mp_params->global_replan_flag = true;
//    mp_params->is_new_global_path = true;
//    mp_params->pruner_global_planner_global_path.reset();
//    mp_params->is_global_plan_success = false;

//    mp_params->last_local_path.clear();
//    mp_params->pruner_local_planner_local_path.reset();
//    mp_params->is_local_plan_success = false;
    PRINT_DEBUG("mp_params->flag_is_task_finished = {}", mp_params->flag_is_task_finished);
    if(!mp_params->flag_is_task_finished)
    {
        PRINT_DEBUG("stop task");
        if(mp_params->p_task_result)
        {
            mp_params->p_task_result->set_value(1);
        }
    }
    mp_params->reset();
    mp_params->flag_is_task_finished = true;
    PRINT_DEBUG("stop finish");
    return true;
}

inline GridMapData TaskNavigation::global_map()
{
    return mp_params->p_client_map->call<RetMsg<GridMapData>>("global_map").msg.data;
}

inline GridMapData TaskNavigation::local_map()
{
    return mp_params->p_client_map->call<RetMsg<GridMapData>>("local_map").msg.data;
}

inline Msg<Pose<FLOAT_T>> TaskNavigation::odom()
{
    return mp_params->p_client_odom->call<RetMsg<Pose<FLOAT_T>>>("location").msg;
}

void TaskNavigation::thread_global_plan()
{
    while (true)
    {
        PRINT_DEBUG("wait condition");
        {
        std::unique_lock<std::mutex> server_lock(mp_params->mtx_condition);
        mp_params->condition.wait(server_lock);
        }
        PRINT_DEBUG("wake up");
        //防止操作系统误唤醒
        if(!mp_params->flag_is_new_task)
        {
            continue;
        }
        //new task
        while (true)
        {
            mp_params->mtx_param.lock();
            const uint32_t periodic_time_ms = mp_params->global_planner_params.global_planner_periodic_time_ms;
            mp_params->mtx_param.unlock();
            PERIODIC_MS_TASK(periodic_time_ms);
            if(mp_params->flag_is_stop_task)
            {
                break;
            }
            global_plan();
        }
    }
}

void TaskNavigation::thread_local_plan()
{
    while (true)
    {
        PRINT_DEBUG("wait condition");
        {
        std::unique_lock<std::mutex> server_lock(mp_params->mtx_condition);
        mp_params->condition.wait(server_lock);
        }
        PRINT_DEBUG("wake up");
        //防止操作系统误唤醒
        if(!mp_params->flag_is_new_task)
        {
            continue;
        }
        //new task
        while (true)
        {
            mp_params->mtx_param.lock();
            const uint32_t periodic_time_ms = mp_params->local_planner_params.local_planner_periodic_time_ms;
            mp_params->mtx_param.unlock();

            PERIODIC_MS_TASK(periodic_time_ms);
            if(mp_params->flag_is_stop_task)
            {
                break;
            }
            local_plan();
        }
    }
}

void TaskNavigation::thread_tracker()
{
    while (true)
    {
        PRINT_DEBUG("wait condition");
        {
        std::unique_lock<std::mutex> server_lock(mp_params->mtx_condition);
        mp_params->condition.wait(server_lock);
        }
        PRINT_DEBUG("wake up");
        //防止操作系统误唤醒
        if(!mp_params->flag_is_new_task)
        {
            continue;
        }
        while (true)
        {
            //PRINT_DEBUG("tracker thread");
            mp_params->mtx_param.lock();
            const uint32_t periodic_time_ms = mp_params->tracker_params.tracker_periodic_time_ms;
            //PRINT_DEBUG("periodic_time_ms = {}", periodic_time_ms);
            mp_params->mtx_param.unlock();
            PERIODIC_MS_TASK(periodic_time_ms);
            if(mp_params->flag_is_stop_task)
            {
                break;
            }
            track_path();
            //PRINT_DEBUG("tracker finish");
        }
    }
}

inline Msg<ControlData> TaskNavigation::control_data()
{
    return mp_params->p_client_robot->call<RetMsg<ControlData>>("feedback_control_data").msg;
}

bool TaskNavigation::is_arrive_goal(const Pose<FLOAT_T> &cur_pose, const Pose<FLOAT_T> &goal,
                                    const FLOAT_T& distance_diff_tolerance, const FLOAT_T& angle_diff_tolerance)
{
    const FLOAT_T dx = cur_pose.position.x - goal.position.x;
    const FLOAT_T dy = cur_pose.position.y - goal.position.y;
    const FLOAT_T distance_diff = hypot(dx, dy);
    FLOAT_T heading_diff = cur_pose.heading_angle - goal.heading_angle;
    heading_diff = radian_to_degree(heading_diff);
    heading_diff = fabs(constraint_angle_d(heading_diff, -180, 180));

    if(distance_diff > distance_diff_tolerance || heading_diff > angle_diff_tolerance)
    {
        return false;
    }
//    PRINT_DEBUG("cur pose({:.3f}, {:.3f}, {:.2f}) close to the goal({:.3f}, {:.3f}, {:.2f})",
//                cur_pose.position.x, cur_pose.position.y, radian_to_degree(cur_pose.heading_angle),
//                goal.position.x, goal.position.y, radian_to_degree(goal.heading_angle));
    return true;
}

bool TaskNavigation::global_plan()
{
    //PRINT_DEBUG("global plan");
    Msg<GridMapData> msg_map_data;
    Msg<Pose<FLOAT_T>> msg_pose_start;
    PathData output_path;
    const Pose<FLOAT_T> pose_start = std::move(odom().data);
    msg_pose_start.data = pose_start;

    mp_params->mtx_param.lock();
    GlobalPlannerParams params(mp_params->global_planner_params);
    Msg<Pose<FLOAT_T>> msg_pose_goal = mp_params->msg_pose_goal;
    const FLOAT_T distance_diff_tolerance = mp_params->distance_diff_tolerance;
    const FLOAT_T angle_diff_tolerance = mp_params->angle_diff_tolerance;
    mp_params->mtx_param.unlock();
    const Pose<FLOAT_T>& pose_goal = msg_pose_goal.data;

    if(is_arrive_goal(pose_start, pose_goal, distance_diff_tolerance, angle_diff_tolerance))
    {
        return true;
    }

    msg_pose_goal.data = pose_goal;
    msg_map_data.data = std::move(global_map());
    std::shared_ptr<MapBase> p_global_map = std::make_shared<GridMapInterface>(msg_map_data.data);
    PRINT_DEBUG("cur pose ({}, {}, {})",
                pose_start.position.x, pose_start.position.y, pose_start.heading_angle * 180 * M_1_PI);
    bool is_need_plan = false;
    bool status = true;
//    uint32_t map_start_pose_x = 0;
//    uint32_t map_start_pose_y = 0;
//    uint32_t map_goal_pose_x = 0;
//    uint32_t map_goal_pose_y = 0;
    //    if(check_environment(mp_global_map, pose_start, pose_goal,
    //                          &map_start_pose_x, &map_start_pose_y,
    //                          &map_goal_pose_x, &map_goal_pose_y))
    {
        if(params.flag_is_global_planner_force_replan)
        {
            PRINT_INFO("receive replan flag");
            is_need_plan = true;
        }
        else if(params.last_global_path.size() < 2)
        {
            PRINT_INFO("last global path < 2");
            is_need_plan = true;
        }
        else
        {
            PathData path = params.last_global_path;
            params.pruner_global_planner_global_path.run(p_global_map, pose_start, &path, 0.5);

            if(!is_path_available(p_global_map, path))
            {
                PRINT_INFO("last global path not available");
                is_need_plan = true;
            }
            else
            {
                //float lateral_error = calculate_cte(pose_start.position, path[0].position, path[1].position);
                FLOAT_T lateral_error = 0.5 * (hypot(pose_start.position.x - path[0].position.x,
                                                     pose_start.position.y - path[0].position.y) +
                                               hypot(pose_start.position.x - path[1].position.x,
                                                     pose_start.position.y - path[1].position.y));
                if(lateral_error > params.global_path_lateral_error)
                {
                    PRINT_WARN("lateral_error = {:.3f}, > {:.3f}", lateral_error, params.global_path_lateral_error);
                    is_need_plan = true;
                }
                else
                {
                    is_need_plan = false;
                    output_path = params.last_global_path;
                }

            }
        }
        //is_need_plan = true;
        if(params.flag_is_global_planner_force_replan)
        {
            is_need_plan = true;
        }
        if(is_need_plan)
        {
            //RECORD_TIME();
            PRINT_INFO("from({:.3f}, {:.3f}, {:.1f}) to ({:.3f}, {:.3f}, {:.1f})",
                       pose_start.position.x, pose_start.position.y, radian_to_degree(pose_start.heading_angle),
                       pose_goal.position.x, pose_goal.position.y, radian_to_degree(pose_goal.heading_angle));

            RetMsg<PathData> ret_msg = mp_params->p_client_global_planner->call<RetMsg<PathData>>("plan", msg_map_data, msg_pose_start, msg_pose_goal);
            bool plan_reslut = false;
            //bool global_replan_flag;
            //bool is_new_global_path;
            if(ret_msg.return_status == RET_SUCCESS)
            {
                if(params.flag_is_enable_global_path_smooth)
                {
                    Msg<GridMapData> msg_map_data2;
                    msg_map_data2.data = std::move(global_map());
                    ret_msg = mp_params->p_client_global_path_smoother->call<RetMsg<PathData>>(
                              "smooth", mp_params->p_client_map->call<RetMsg<GridMapData>>("global_map").msg,
                              ret_msg.msg);
                }
                mp_params->p_client_robot->call("set_global_planner_path", ret_msg.msg);
                plan_reslut = true;
                params.last_global_path = ret_msg.msg.data;
                //global_replan_flag = false;
                params.flag_is_new_global_path = true;
                params.pruner_global_planner_global_path.reset();
            }
            status = plan_reslut;
        }
    }

    mp_params->mtx_param.lock();
    mp_params->global_planner_params = params;
    mp_params->mtx_param.unlock();

    return status;
}

bool TaskNavigation::local_plan()
{
    bool status = false;
    mp_params->mtx_param.lock();
    LocalPlannerParams params = mp_params->local_planner_params;
    GlobalPlannerParams global_params = mp_params->global_planner_params;
    const Msg<Pose<FLOAT_T>> msg_pose_goal = mp_params->msg_pose_goal;
    const FLOAT_T distance_diff_tolerance = mp_params->distance_diff_tolerance;
    const FLOAT_T angle_diff_tolerance = mp_params->angle_diff_tolerance;
    mp_params->mtx_param.unlock();
    Msg<GridMapData> msg_map_data;
    Msg<Pose<FLOAT_T>> msg_pose_start = std::move(odom());
    const Pose<FLOAT_T> &pose_start = msg_pose_start.data;
    const Pose<FLOAT_T> &pose_goal = msg_pose_goal.data;

    if(is_arrive_goal(pose_start, pose_goal, distance_diff_tolerance, angle_diff_tolerance))
    {
        return true;
    }

    PathData output_path;
    msg_map_data.data = std::move(local_map());
    std::shared_ptr<MapBase> p_map = std::make_shared<GridMapInterface>(msg_map_data.data);
    PRINT_DEBUG("cur pose ({}, {}, {} to goal ({}, {}, {})",
                pose_start.position.x, pose_start.position.y, pose_start.heading_angle * 180 * M_1_PI,
                pose_goal.position.x, pose_goal.position.y, pose_goal.heading_angle * 180 * M_1_PI);

    bool plan_result = true;
    PathData global_path = global_params.last_global_path;
    bool force_replan = params.flag_is_local_planner_force_replan;
    bool is_use_global_path = params.flag_is_local_planner_use_global_path;
    bool is_global_path_available = false;
    bool is_local_path_available = true;
    Pose<float> local_plan_goal = pose_start;
    bool is_need_plan = false;
    std::vector<Pose<float>> local_path;
    std::vector<float> global_path_frenet_s;
    std::vector<float> global_path_frenet_d;
    std::vector<float> global_path_x;
    std::vector<float> global_path_y;
    std::vector<float> global_path_heading;
    //p_path->clear();
    //*p_is_replan = true;
    local_path = params.last_local_path;
    if(global_params.flag_is_new_global_path)
    {
        //PRINT_INFO("recv new global plan");
        //PRINT_DEBUG("global size = {}", global_path.size());
//        mp_params->last_local_planner_target = pose_start;
//        mp_params->pruner_local_planner_global_path.reset();
//        mp_params->pruner_local_planner_local_path.reset();
//        mp_params->is_new_global_path = false;

        params.last_local_planner_target = pose_start;
        params.pruner_local_planner_global_path.reset();
        params.pruner_local_planner_local_path.reset();
        global_params.flag_is_new_global_path = false;
        if(global_path.size() < 2)
        {
            //PRINT_ERROR("global path is empty, skip local plan, return\n");
            return false;
        }
    }

    //if(!mp_params->is_new_global_path)
    {
        //裁剪全局规划出来的路径，得到全局规划在局部地图中的路径段
        //PRINT_INFO("pruner_local_planner_global_path start");
        params.pruner_local_planner_global_path.run(p_map, pose_start, &global_path);
        //PRINT_INFO("pruner_local_planner_global_path end");
        //PRINT_INFO("pruner_local_planner_local_path start");
        params.pruner_local_planner_local_path.run(p_map, pose_start, &local_path, 0.5);
        //PRINT_INFO("pruner_local_planner_local_path end");
        //PRINT_DEBUG("local_path.size = {}", local_path.size());
        if(global_path.size() < 2)
        {
            //PRINT_ERROR("global path is empty, skip local plan, return\n");
            return false;
        }

        if(is_path_available(p_map, local_path))
        {
            //float lateral_error = calculate_cte(pose_start.position, local_path[0].position, local_path[1].position);
            float lateral_error = 0.5 * (hypot(pose_start.position.x - local_path[0].position.x,
                                               pose_start.position.y - local_path[0].position.y) +
                                         hypot(pose_start.position.x - local_path[1].position.x,
                                               pose_start.position.y - local_path[1].position.y));
            //if local plan not to goal && length too short
            //            PRINT_DEBUG("last target pose = {},{},{}, pose_goal = {} {} {}",
            //                        mp_params->last_local_planner_target.position.x, mp_params->last_local_planner_target.position.y, mp_params->last_local_planner_target.heading_angle,
            //                        pose_goal.position.x, pose_goal.position.y, pose_goal.heading_angle);
            if(local_path.back() != pose_goal)
            {

                double total_distance = 0;
                for(int i = 1; i < local_path.size(); ++i)
                {
                    const float dx = local_path[i].position.x - local_path[i-1].position.x;
                    const float dy = local_path[1].position.y - local_path[i-1].position.y;
                    total_distance += hypot(dx, dy);
                    if(total_distance > params.local_planner_replan_distance)
                    {
                        break;
                    }
                }

                if(total_distance <= params.local_planner_replan_distance)
                {
                    is_local_path_available = false;
                    //PRINT_WARN("local path too short");
                }
            }
            if(lateral_error > params.local_path_lateral_error)
            {
                //PRINT_WARN("lateral_error = {:.3f}, > {:.3f} skip local path", lateral_error, mp_params->local_path_lateral_error);
                is_local_path_available = false;
            }
        }
        else
        {
            is_local_path_available = false;
            //PRINT_WARN("local path unaviliable");
        }

        if(params.flag_is_local_planner_use_global_path)
        {
            is_global_path_available = true;
            //global_path = interpolate_path(global_path, p_map->resolution());
            if(is_path_available(p_map, global_path))
            {
                //PRINT_INFO("use global path");
                //float lateral_error = calculate_cte(pose_start.position, global_path[0].position, global_path[1].position);
                float lateral_error = 0.5 * (hypot(pose_start.position.x - global_path[0].position.x,
                                                   pose_start.position.y - global_path[0].position.y) +
                                             hypot(pose_start.position.x - global_path[1].position.x,
                                                   pose_start.position.y - global_path[1].position.y));
                if(lateral_error > global_params.global_path_lateral_error)
                {
                    //PRINT_WARN("lateral_error = {:.3f}, > {:.3f} skip global path", lateral_error, mp_params->local_path_lateral_error);
                    is_global_path_available = false;
                }
            }
            else
            {
                is_global_path_available = false;
                //PRINT_WARN("global path unaviliable");
            }
        }
        if(force_replan)
        {
            goto NEED_PLAN;
        }

        if(is_use_global_path && is_global_path_available && is_local_path_available)
        {
            //params.flag_is_local_plan_success = true;
            //select best path
            output_path = select_best_path(p_map, global_path, local_path, pose_start);
            // 暂时先用local path
            //*p_is_replan = true;
            //*p_path = local_path;
            //PRINT_INFO("use last local plan, return\n");

//            return true;
        }
        else if(is_use_global_path && is_global_path_available)
        {
            //params.flag_is_local_plan_success = true;
            output_path = global_path;
            //*p_is_replan = true;
            if(global_path.back() == pose_goal)
            {
                //mp_params->last_local_path = global_path;
                params.last_local_planner_target = pose_goal;
            }
            //m_last_local_path = global_path;
            PRINT_INFO("use global path, return\n");
//            Msg<ControlData> msg_cur_control_data = control_data();
//            Msg<PathData> msg_path_data;
//            msg_path_data.data = output_path;
//            mp_params->p_client_tracker->call("set_path", msg_path_data, msg_cur_control_data);
//            return true;
        }
        else if(is_local_path_available)
        {
            //*p_is_replan = true;
            //params.flag_is_local_plan_success = true;
            output_path = local_path;
            PRINT_INFO("use last local plan, return");
//            Msg<ControlData> msg_cur_control_data = control_data();
//            Msg<PathData> msg_path_data;
//            msg_path_data.data = output_path;
//            mp_params->p_client_tracker->call("set_path", msg_path_data, msg_cur_control_data);
//            return true;
        }
        else
        {
            params.flag_is_local_plan_success = false;
            PRINT_WARN("last local and global path are both unavailable, replan");
            goto NEED_PLAN;
        }
        //if last path enable
        params.flag_is_local_plan_success = true;
        Msg<ControlData> msg_cur_control_data = control_data();
        Msg<PathData> msg_path_data;
        msg_path_data.data = output_path;
        mp_params->p_client_tracker->call("set_path", msg_path_data, msg_cur_control_data);
        mp_params->p_client_robot->call("set_local_planner_path", msg_path_data);
        mp_params->mtx_param.lock();
        mp_params->local_planner_params = params;
        mp_params->global_planner_params = global_params;
        mp_params->mtx_param.unlock();

        return true;

    }
//    else
//    {
//        PRINT_INFO("recv new global plan");
//        PRINT_DEBUG("global size = {}", global_path.size());
//        mp_params->last_local_planner_target = pose_start;
//        mp_params->pruner_local_planner_global_path.reset();
//        mp_params->pruner_local_planner_local_path.reset();
//        m_is_new_global_path = false;
//        mp_params->pruner_local_planner_global_path.run(p_map, pose_start, &global_path);
//        //mp_params->pruner_local_planner_local_path.run(p_map, pose_start, &local_path, 0.5);
//        if(global_path.size() < 2)
//        {
//            PRINT_ERROR("global path is empty, skip local plan, return\n");
//            return false;
//        }
//        PRINT_DEBUG("new global path, goto need-plan");
//        goto NEED_PLAN;
//    }

NEED_PLAN:
    if(p_map->world_collision_detect(global_path.back()))
    {
        //RECORD_TIME("sd");
        //PRINT_DEBUG("last goal pose collision, pose {} {}", global_path.back().position.x, global_path.back().position.y);
        global_path_frenet_s.resize(global_path.size());
        global_path_frenet_d.resize(global_path.size());
        global_path_x.resize(global_path.size());
        global_path_y.resize(global_path.size());
        global_path_heading.resize(global_path.size());
        for(int i = 0; i < global_path_x.size(); ++i)
        {
            global_path_x[i] = global_path[i].position.x;
            global_path_y[i] = global_path[i].position.y;
            global_path_heading[i] = global_path[i].heading_angle;
        }
        for(int i = 0; i < global_path_x.size(); ++i)
        {
            std::vector<float> frenet_s_d = get_frenet(global_path_x[i], global_path_y[i], global_path_heading[i],
                                                       global_path_x, global_path_y);
            global_path_frenet_s[i] = frenet_s_d[0];
            global_path_frenet_d[i] = frenet_s_d[1];
        }
        const float road_w = 2;
        const float road_step = 0.3;
        float goal_s = 0;
        float goal_d = 0;
        uint8_t cost = p_map->obstacles_cost() - 1;
        std::vector<float> goal_pose;
        for(float d = -road_w; d <= road_step; d+=road_step)
        {
            std::vector<float> temp_goal_pose = get_xy(global_path_frenet_s.back(), d, global_path_frenet_s, global_path_x, global_path_y);
            if(p_map->world_cost(temp_goal_pose[0], temp_goal_pose[1]) < cost)
            {
                cost = p_map->world_cost(temp_goal_pose[0], temp_goal_pose[1]);
                goal_pose = temp_goal_pose;
            }
        }

        if(cost == p_map->obstacles_cost() - 1)
        {
            //PRINT_ERROR("now place to plan, return");
            return false;
        }
        else
        {
            //PRINT_DEBUG("new goal pose: {} {}, cost = {}", goal_pose[0], goal_pose[1], cost);
            local_plan_goal.position.x = goal_pose[0];
            local_plan_goal.position.y = goal_pose[1];
            local_plan_goal.heading_angle = global_path_heading.back();
            //is_need_plan = true;
            //goto NEED_PLAN;
        }
    }
    else
    {
        local_plan_goal = global_path.back();
    }
    //if(is_need_plan)
    {
        //RECORD_TIME();
//        PRINT_DEBUG("from ({}, {}, {}) to ({}, {}, {})",
//                    pose_start.position.x, pose_start.position.y, pose_start.heading_angle * 180 * M_1_PI,
//                    local_plan_goal.position.x, local_plan_goal.position.y, local_plan_goal.heading_angle * 180 * M_1_PI);
        //plan
        Msg<Pose<FLOAT_T>> msg_local_plan_goal;
        msg_local_plan_goal.data = local_plan_goal;
        Msg<PathData> msg_global_path;
        msg_global_path.data = global_path;
        mp_params->p_client_local_planner->call("set_reference_path", msg_global_path);
        RetMsg<PathData> ret_msg = mp_params->p_client_local_planner->call<RetMsg<PathData>>("plan", msg_map_data, msg_pose_start, msg_local_plan_goal);


        //bool plan_reslut = false;

          if(ret_msg.return_status == RET_SUCCESS)
          {
              params.flag_is_local_plan_success = true;
              Msg<GridMapData> msg_map_data2;
              msg_map_data2.data = std::move(local_map());
              if(params.flag_is_enable_local_path_smooth)
              {
                  PRINT_DEBUG("before smooth");
//                  for(int i = 0; i != ret_msg.msg.data.size(); ++i)
//                  {
//                      const Pose<FLOAT_T> &pose = ret_msg.msg.data[i];
//                      printf("[%d] %.2f, %.2f, %.2f\n", i, pose.position.x, pose.position.y, radian_to_degree(pose.heading_angle));
//                  }
                   ret_msg = mp_params->p_client_local_path_smoother->call<RetMsg<PathData>>(
                               "smooth", msg_map_data2, ret_msg.msg);
//                   PRINT_DEBUG("smooth result = {}", ret_msg.return_status);
//                   PRINT_DEBUG("after smooth");
//                   for(int i = 0; i != ret_msg.msg.data.size(); ++i)
//                   {
//                       const Pose<FLOAT_T> &pose = ret_msg.msg.data[i];
//                       printf("[%d] %.2f, %.2f, %.2f\n", i, pose.position.x, pose.position.y, radian_to_degree(pose.heading_angle));
//                   }
              }

//              Msg<ControlData> msg_cur_control_data = control_data();
//              mp_params->p_client_tracker->call("set_path", ret_msg.msg, msg_cur_control_data);
//              mp_params->p_client_robot->call("set_local_planner_path", ret_msg.msg);
//              params.last_local_planner_target = local_plan_goal;
//              params.last_local_path = ret_msg.msg.data;
//              params.pruner_local_planner_local_path.reset();
          }
          else
          {
              ret_msg.msg.data.clear();
              params.flag_is_local_plan_success = false;
              plan_result = false;
              mp_params->p_client_robot->call("stop");
              PRINT_ERROR("local_plan_failed");
          }

          Msg<ControlData> msg_cur_control_data = control_data();
          mp_params->p_client_tracker->call("set_path", ret_msg.msg, msg_cur_control_data);
          mp_params->p_client_robot->call("set_local_planner_path", ret_msg.msg);
          params.last_local_planner_target = local_plan_goal;
          params.last_local_path = ret_msg.msg.data;
          params.pruner_local_planner_local_path.reset();

          //PRINT_DEBUG("call back finished");
    }
    mp_params->mtx_param.lock();
    mp_params->local_planner_params = params;
    mp_params->global_planner_params = global_params;
    mp_params->mtx_param.unlock();
    return plan_result;
}

bool TaskNavigation::track_path()
{
    mp_params->mtx_param.lock();
    LocalPlannerParams local_params = mp_params->local_planner_params;
    GlobalPlannerParams global_params = mp_params->global_planner_params;
    const Msg<Pose<FLOAT_T>> msg_pose_goal = mp_params->msg_pose_goal;
    bool flag_is_stop_task = mp_params->flag_is_stop_task;
    TrackerParams params = mp_params->tracker_params;
    const FLOAT_T distance_diff_tolerance = mp_params->distance_diff_tolerance;
    const FLOAT_T angle_diff_tolerance = mp_params->angle_diff_tolerance;
    mp_params->mtx_param.unlock();
    if(local_params.flag_is_local_plan_success)
    {
        //RECORD_TIME();
        Msg<Pose<FLOAT_T>> msg_location = odom();
        if(is_arrive_goal(msg_location.data, mp_params->msg_pose_goal.data, distance_diff_tolerance, angle_diff_tolerance))
        {
            RetMsg<bool> ret_msg = mp_params->p_client_tracker->call<RetMsg<bool>>("is_goal_reached");
            if(ret_msg.msg.data == true)
            {
                flag_is_stop_task = true;
                mp_params->p_client_tracker->call("set_stop_signal");
                mp_params->p_client_robot->call("stop");
                mp_params->flag_is_task_finished = true;
                if(mp_params->p_task_result)
                {
                    mp_params->p_task_result->set_value(0);
                }
                return true;
            }
        }
        Msg<ControlData> msg_cur_control_data = control_data();
        RetMsg<ControlData> msg_control_cmd = mp_params->p_client_tracker->call<RetMsg<ControlData>>("scroll_calculate",
                                                               FLOAT_T(params.tracker_periodic_time_ms * 0.001),
                                                               msg_location, msg_cur_control_data);
        //PRINT_ERROR("start set cmd");
        if(msg_control_cmd.return_status == RET_SUCCESS)
        {
           Msg<GridMapData> msg_map_data;
           //PRINT_ERROR("start get local map");
           msg_map_data.data = local_map();
           //PRINT_ERROR("end get local map");
           Msg<Pose<FLOAT_T>> msg_pose = std::move(odom());
//           PRINT_DEBUG("set control data: {:.2f}, {:.2f}", msg_control_cmd.msg.data.velocity,
//                       radian_to_degree(msg_control_cmd.msg.data.steer_angle));
           //PRINT_ERROR("start set control");
           mp_params->p_client_robot->call("set_control_data", msg_map_data, msg_control_cmd.msg, msg_pose);
           //PRINT_ERROR("end set control");
//           mp_params->p_rpc_server->publish("tracker_velocity", msg_control_cmd.msg);
        }
        //PRINT_ERROR("end set cmd");
    }
    else
    {
        mp_params->p_client_tracker->call("set_stop_signal");
        mp_params->p_client_robot->call("stop");
    }
    return true;
}


const PathData& TaskNavigation::select_best_path(std::shared_ptr<MapBase> p_map,
                                                 const PathData &path1, const PathData &path2, const Pose<float>& cur_pose)
{
    //    const float cost_angle;
    const float cost_smooth = 0.5;
    const float cost_obstacles = 0.6;
    const float cost_length = 0.1;
    const float cost_offset = 0.35;

    std::vector<const PathData*> p_path_list;
    p_path_list.emplace_back(&path1);
    p_path_list.emplace_back(&path2);

    size_t best_path_index = 0;
    float best_score = std::numeric_limits<float>::max();
    std::vector<std::vector<FLOAT_T>> path_score_list;
    path_score_list.resize(p_path_list.size());
    const float obstacles_1 = 1.0f / p_map->obstacles_cost();
    for(int path_index = 0; path_index < p_path_list.size(); ++path_index)
    {
        const PathData &path = *p_path_list[path_index];
        float path_len = 0;
        float path_angle = 0;
        float path_map_cost = 0;
        float path_offset = 0;

        const size_t path_size = path.size();
        for(int i = 1; i < path_size; ++i)
        {
            path_len += hypot(path[i].position.x - path[i-1].position.x, path[i].position.y - path[i-1].position.y);
            path_angle += M_1_PI * fabs(constraint_angle_r(path[i].heading_angle - path[i-1].heading_angle, -M_PI, M_PI));
            path_map_cost += obstacles_1 * p_map->world_cost(path[i].position.x, path[i].position.y);
        }
        path_angle = path_angle / path_size;
        //path_angle = exp(path_angle);
        path_map_cost /= path_size;
        path_map_cost = exp(path_map_cost);
        path_offset = fabs(calculate_cte(cur_pose.position, path[0].position, path[1].position));
        path_offset = exp(path_offset);
        path_score_list[path_index] = {path_len, path_angle, path_angle, path_offset};

        PRINT_DEBUG("path[{}]: len: {}, angle: {}, map_cost: {}, offset: {}", path_index,
                    path_len, path_angle, path_map_cost, path_offset);
    }
    FLOAT_T total_path_len = 0;
    FLOAT_T total_path_angle = 0;
    FLOAT_T total_path_map_cost = 0;
    FLOAT_T total_path_offset = 0;
    for(int path_index = 0; path_index < path_score_list.size(); ++path_index)
    {
        total_path_len += path_score_list[path_index][0];
        total_path_angle += path_score_list[path_index][1];
        total_path_map_cost += path_score_list[path_index][2];
        total_path_offset += path_score_list[path_index][3];
    }
    for(int path_index = 0; path_index < path_score_list.size(); ++path_index)
    {
        float total_score = -path_score_list[path_index][0] / total_path_len * cost_length +
                            path_score_list[path_index][1] / total_path_angle * cost_smooth +
                            path_score_list[path_index][2] / total_path_map_cost * cost_obstacles +
                            //path_score_list[path_index][3] / total_path_offset * cost_offset;
                            path_score_list[path_index][3] * cost_offset;
        PRINT_DEBUG("path[{}] - score: {}", path_index, total_score);
        if(total_score < best_score)
        {
            best_score = total_score;
            best_path_index = path_index;
        }
    }
    PRINT_DEBUG("select path {}", best_path_index);
    return *p_path_list[best_path_index];
}

static VectorX2<float> closest_point_on_line(const VectorX2<float> &cur_pose,
                                             const VectorX2<float> &a,
                                             const VectorX2<float> &b)
{
    VectorX2<float> point;
    VectorX2<float> a_p = cur_pose - a;
    VectorX2<float> a_b = b - a;
    float len_square = a_b.x * a_b.x + a_b.y * a_b.y;
    float ab_ap_product = a_b.x * a_p.x + a_b.y * a_p.y;
    float distance = ab_ap_product / len_square;
    if(distance < 0)
    {
        point = a;
    }
    else if(distance > 1)
    {
        point = b;
    }
    else
    {
        point = a_b;
        point.x *= distance;
        point.y *= distance;
        point = a + point;
    }
    return point;
}

static float calculate_cte(const VectorX2<float> cur_point,
                           const VectorX2<float> point_from,
                           const VectorX2<float> point_to)
{
    VectorX2<float> cloest_point = closest_point_on_line(cur_point, point_from, point_to);
    const float dx = cur_point.x - cloest_point.x;
    const float dy = cur_point.y - cloest_point.y;
    float cte = hypot(dx, dy);
    //determine if CTE is negative or positive so we can steer in the direction we need
    //Is the car to the right or to the left of the upcoming waypoint
    VectorX2<float> to_cur_vec = cur_point - point_from;
    VectorX2<float> to_point_vec = point_to - point_from;
    float angle1 = atan2(to_cur_vec.y, to_cur_vec.x);
    float angle2 = atan2(to_point_vec.y, to_point_vec.x);
    float angle_diff = constraint_angle_r(angle1 - angle2, -M_PI, M_PI);
    if(angle_diff < 0)
    {
        cte *= -1.0;
    }
    return cte;
}

}
