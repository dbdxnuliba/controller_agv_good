//#if 1
//// C HEADER
//#include <stdio.h>
//#include <stdlib.h>
//#include <memory.h>
//#include <math.h>
//#include <time.h>
//// C++ HEADER
//#include <iostream>
//#include <vector>
//#include <string>
//#include <thread>
//#include <mutex>
//#include <fstream>
//#include <iomanip>
//#include <exception>
//#include <sstream>
//#include <atomic>
//// LINUX C HEADER
//#include <unistd.h>
//#include <sys/time.h>
//#include <errno.h>
//#include <sys/timerfd.h>
//#include <signal.h>
////ROS HEADER

//// CUSTOM HEADER
//#include "common/spdlog/spdlog.h"
//#include "common/time_keeper.h"
//#include "common/common.h"
//#include "common/print.h"
//#include "common/periodic_task.h"
//#include "modules/model/akermann_model.h"
//#include "interpolater.h"
////#include "body_control.h"
//#include "common/json.hpp"
//#include "modules/msg/nng.h"
//#include "modules/tracker/pid/pid.h"

//#include "mpc.h"
//#include "track.h"
//#include <nlohmann/json.hpp>

//using namespace std;
//using MPC = mpcc::MPC;
//using Track = mpcc::Track;
//using PathToJson = mpcc::PathToJson;
//using State = mpcc::State;
//using MPCReturn = mpcc::MPCReturn;

//static std::atomic<bool> IS_COLLISION(false);
//static std::recursive_mutex MTX_CONTROL;
//static std::recursive_mutex MTX_POSE;
//static Pose<double> CUR_POSE;

//static double actual_linear_velocity = 0.0;
//static double actual_head_angle = 0.0;
//static std::vector<Pose<double>> last_path;
//std::map<NNG_ID, std::string> NNG_URLS;

//static double kControlCycle = 0.12;//控制周期为30ms
//static std::atomic<int> path_size(0);
//static bool is_open_json = false;//判断是否打开json文件,当已经打开置为true

//bool IsNeedSetPath(const std::vector<Pose<double>> &last_path,const std::vector<Pose<double>> &current_path)
//{
//  int index = last_path.size() - current_path.size();
//  //说明第一次收到路径
//  if(last_path.size() == 0)
//  {
//    return true;
//  }
//  //如果终点不相同,需要设置路径
//  if(last_path.at(last_path.size()-1).heading_angle != current_path.at(current_path.size()-1).heading_angle ||
//     last_path.at(last_path.size()-1).position.x    != current_path.at(current_path.size()-1).position.x ||
//     last_path.at(last_path.size()-1).position.y    != current_path.at(current_path.size()-1).position.y)
//  {
//    return true;
//  }
//  //如果终点相同,但index<0,说明路径一定有变化
//  if(index<0)
//  {
//    return true;
//  }
//  else
//  {
//    for(int i = index;i<last_path.size();i++)
//    {
//      if(last_path.at(i).heading_angle != current_path.at(i).heading_angle ||
//         last_path.at(i).position.x    != current_path.at(i).position.x ||
//         last_path.at(i).position.y    != current_path.at(i).position.y)
//      {
//        return true;
//      }
//    }
//    return false;
//  }
//}

//void ComputeControl(const double dt, MPC *mpc, int sock)
//{
//  //1获取机器人当前位置
//  MTX_POSE.lock();
//  Pose<double> cur_pose = CUR_POSE;
//  MTX_POSE.unlock();
//  //2 定义控制量
//  double velocity = 0;
//  double steer_angle = 0;
//  //2.1 如果前方没有障碍物，规划控制量
//  if(!IS_COLLISION)
//  {
//    //2.1.1 更新控制量
//    static State x0;
//    MTX_CONTROL.lock();
//    //通过run once 计算控制量，并进行相关变量的更新
//    if(is_open_json == false)
//    {
//      x0.X     = cur_pose.position.x;
//      x0.Y     = cur_pose.position.y;
//      x0.phi   = cur_pose.heading_angle;
//      x0.vx    = 0.1;
//      x0.vy    = 0.0;
//      x0.r     = 0.0;
//      x0.s     = 0.0;
//      x0.D     = 0.5;
//      x0.delta = actual_head_angle;
//      x0.vs    = 0.1;
//      is_open_json = true;
//    }
//    else
//    {
//      x0.X     = cur_pose.position.x;
//      x0.Y     = cur_pose.position.y;
//      x0.phi   = cur_pose.heading_angle;
//      x0.vs    = actual_linear_velocity;
//      x0.delta = actual_head_angle;
//    }
//    MPCReturn mpc_return = mpc->RunMPC(x0);
//    velocity             = mpc_return.mpc_horizon.at(1).xk.vs;
//    steer_angle          = mpc_return.mpc_horizon.at(1).xk.delta;

//    //std::cout<<"当前时刻的状态量"<<std::endl;
//    //mpc->PrintState(mpc_return.mpc_horizon.at(1).xk);
//    //std::cout<<"优化时间:"<<mpc_return.time_total<<std::endl;
//    MTX_CONTROL.unlock();
//    //2.1.2 根据当前状态和控制更新一次CUR_POSE,防止拿不到里程数据
//    //MTX_POSE.lock();
//    x0 = mpc_return.mpc_horizon.at(1).xk;
//    //MTX_POSE.unlock();
//    //说明当前路径已经结束,不需要进行mpc求解
//    if(velocity == 0 && steer_angle == 0)
//    {
//      path_size = 0;
//      last_path.clear();
//      std::cout<<"到达终点,路径长度置为0"<<std::endl;
//    }
//  }
//  //2.2 检测到障碍物
//  else
//  {
//    PRINT_ERROR("detect collision");
//  }
//  //3.发布控制量
//  std::vector<double> control_data;
//  control_data.resize(3);
//  control_data[0] = dt;
//  control_data[1] = velocity;
//  control_data[2] = steer_angle;

//  if(!send_data(sock, control_data))
//  {
//    spdlog::error("send msg error");
//  }

//}
//void VelocityControlThread(MPC *mpc, const string & url)
//{
//  int sock = create_socket_pub(url);
//  try
//  {
//    while(1)
//    {
//      if(path_size == 0)
//      {
//        //std::cout<<"mpc控制器未收到路径信息,不需要执行计算控制量的线程"<<std::endl;
//        std::vector<double> control_data;
//        control_data.resize(3);
//        control_data[0] = 0.0;
//        control_data[1] = 0.0;
//        control_data[2] = 0.0;
//        if(!send_data(sock, control_data))
//        {
//          spdlog::error("send msg error");
//        }
//        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//      }
//      else
//      {
//        PERIODIC_TASK(kControlCycle * 1000);//TASK_PERIOD_MS = 30ms
//        ComputeControl(kControlCycle, mpc, sock);
//      }
//    }
//  }
//  catch(...)
//  {
//    std::vector<double> control_data;
//    control_data.resize(3);
//    control_data[0] = 0.0;
//    control_data[1] = 0.0;
//    control_data[2] = 0.0;
//    if(!send_data(sock, control_data))
//    {
//      spdlog::error("send msg error");
//    }
//  }
//}
//static void OnPathUpdate(const std::string &url, Track *track, MPC *mpc)
//{
//  int sock = create_socket_sub(url);
//  while (1)
//  {
//    std::vector<Pose<double>> path;
//    if(recv_data(sock, &path))
//    {
//      if(path.size()>1)
//      {
//        MTX_CONTROL.lock();
//        if(!IsNeedSetPath(last_path,path))
//        {
//          //std::cout<<"收到不需要处理的路径"<<std::endl;
//        }
//        else
//        {
//          std::cout<<"收到需要处理的路径"<<std::endl;
//          track->SetTrack(path);
//          mpcc::TrackPos track_pos = track->GetTrack();
//          mpc->SetTrack(track_pos.X,track_pos.Y);
//          path_size = path.size();

//        }
//        last_path = path;
//        MTX_CONTROL.unlock();
//      }
//      else
//      {
//        std::cout<<"线路太短"<<std::endl;
//        path_size = 0;
//      }
//    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(1));
//  }
//}
//static void OnOdomUpdate(const std::string &url)
//{
//  int sock = create_socket_sub(url);
//  for (;;)
//  {
//    char *buf = nullptr;
//    int bytes = nn_recv(sock, &buf, NN_MSG, 0);
//    if (bytes < 0) {
//      spdlog::error("%s nn_recv", url);
//    }

//    Pose<double> odom_pose;
//    if(recv_data(sock, &odom_pose))
//    {
//      MTX_POSE.lock();
//      CUR_POSE = odom_pose;
//      MTX_POSE.unlock();
//    }
//    //里程计消息有抖动，不要过于相信，后续考虑添加滤波处理模块, 因此这边改为每100ms获取一次
//    //std::this_thread::sleep_for(std::chrono::milliseconds(200));
//  }
//}

//static void OnCollisionDetection(const std::string &url)
//{
//  int sock = create_socket_sub(url);
//  while (1)
//  {
//    bool is_collision;
//    if(recv_data(sock, &is_collision))
//    {
//      IS_COLLISION = is_collision;
//    }
//    else
//    {
//      IS_COLLISION = false;
//    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(1));
//  }
//}
//static void thread_sub_actual_velocity(const std::string &url)
//{
//  int sock = create_socket_sub(url);
//  for (;;)
//  {
//    std::vector<double> control_data;
//    if(recv_data(sock, &control_data))
//    {
//      MTX_CONTROL.lock();
//      actual_linear_velocity = control_data[1];
//      actual_head_angle      = control_data[2];
//      //std::cout<<"后轮实际线速度:"<<actual_linear_velocity<<"前轮实际转角:"<<actual_head_angle<<std::endl;
//      MTX_CONTROL.unlock();
//    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(1));
//  }
//}

//int main(int argc, char **argv)
//{
//  std::cout<<"运动控制器正常启动"<<std::endl;
//  //1.读取json文件,并初始化mpc控制器
//  is_open_json = false;
//  last_path.clear();
//  if(!import_nng_configs("/home/caopan/bz_robot/params/nano_msg.json", &NNG_URLS))
//  {
//      exit(-1);
//  }

//  std::ifstream iConfig("/home/caopan/bz_robot/params/mpc_params/config.json");
//  if(iConfig.is_open())
//  {
//    std::cout<<"connect config json success"<<std::endl;
//  }
//  nlohmann::json jsonConfig;
//  iConfig >> jsonConfig;
//  PathToJson json_paths {jsonConfig["model_path"],
//                         jsonConfig["cost_path"],
//                         jsonConfig["bounds_path"],
//                         jsonConfig["track_path"],
//                         jsonConfig["normalization_path"]};
//  kControlCycle = jsonConfig["Ts"];
//  MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],kControlCycle,json_paths);
//  Track track;
//  //std::thread传入参数（）,第1个参数为函数名，后面要传入函数的参数
//  std::vector<std::thread*> thread_list;
//  thread_list.push_back(new std::thread(OnOdomUpdate, NNG_URLS[NNG_ID_NAV_ODOM_SUB]));
//  thread_list.push_back(new std::thread(OnPathUpdate, NNG_URLS[NNG_ID_NAV_LOCAL_PLAN_SUB], &track, &mpc));
//  thread_list.push_back(new std::thread(VelocityControlThread, &mpc, NNG_URLS[NNG_ID_ROBOT_TRACK_VELOCITY_PUB]));
//  thread_list.push_back(new std::thread(OnCollisionDetection, NNG_URLS[NNG_ID_NAV_COLLISION_DETECTION_SUB]));
//  thread_list.push_back(new std::thread(thread_sub_actual_velocity, NNG_URLS[NNG_ID_ROBOT_ACTUAL_VELOCITY_SUB]));
//  //detach实现子线程与主线程的分离，
//  for(auto it = thread_list.begin(); it != thread_list.end(); ++it)
//  {
//    (*it)->detach();
//  }
//  //通过循环保持进程，使得thread_list的线程正常运行
//  while (1)
//  {
//    std::this_thread::sleep_for(std::chrono::seconds(10));
//  }
//  std::cout<<"运动控制器正常结束"<<std::endl;
//  return 0;
//}
//#endif
