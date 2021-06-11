#include "common/common.h"
#include "common/print.h"
#include "common/timer/timer.h"
#include "modules/main/collision_detector_main.h"
#include "modules/main/localization_main.h"
#include "modules/main/map_main.h"
#include "modules/main/planner_main.h"
#include "modules/main/robot_main.h"
#include "modules/main/service_main.h"
#include "modules/main/smoother_main.h"
#include "modules/main/task_main.h"
#include "modules/main/tracker_main.h"
#include "modules/main/ros_plugins/plugins_ros_interface.h"

int main(int argc, char **argv)
{
    using namespace bz_robot;
    init_log("bz_robot");
    for(int i = 0; i != argc; ++i)
    {
        PRINT_INFO("cmd[{}]:\t{}", i, argv[i]);
    }
    std::vector<std::thread*> thread_list;
    thread_list.push_back(new std::thread(&bz_robot::timer::Timer::run, &bz_robot::timer::GetTimer::timer()));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_list.push_back(new std::thread(localization_module_main, argc, argv));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_list.push_back(new std::thread(collision_detector_module_main, argc, argv));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_list.push_back(new std::thread(map_module_main, argc, argv));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_list.push_back(new std::thread(planner_module_main, argc, argv));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_list.push_back(new std::thread(robot_module_main, argc, argv));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_list.push_back(new std::thread(smoother_module_main, argc, argv));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_list.push_back(new std::thread(tracker_module_main, argc, argv));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_list.push_back(new std::thread(task_module_main, argc, argv));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_list.push_back(new std::thread(service_module_main, argc, argv));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    thread_list.push_back(new std::thread(plugins_ros_interface_module_main, argc, argv));
    for(auto it = thread_list.begin(); it != thread_list.end(); ++it)
    {
        (*it)->join();
    }

    PRINT_ERROR("bz_robot exit!");
    return 0;
}
