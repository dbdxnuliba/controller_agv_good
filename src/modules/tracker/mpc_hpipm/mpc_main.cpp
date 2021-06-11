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
//#include "model/ackermann_model.h"
//#include "common/periodic_task.h"
////#include "body_control.h"
//#include "common/json.hpp"
//#include "modules/tracker/pid/pid.h"

//#include "mpc/model_predictive_control.h"
//#include "deal_with_track.h"

//using namespace std;
//using namespace mpc;
//static std::atomic<bool> IS_COLLISION(false);
//static std::recursive_mutex MTX_CONTROL;
//static std::recursive_mutex MTX_POSE;
//static Pose<float> CUR_POSE;

//static float actual_linear_velocity = 0.0;
//static float actual_head_angle = 0.0;

//static std::vector<Pose<float>> last_path;
//static MpcResult mpc_result_;
//std::map<NNG_ID, std::string> NNG_URLS;

//static float kControlCycle = 0.21;//控制周期为30ms
//static std::atomic<int> path_size(0);

//bool IsNeedSetPath(const std::vector<Pose<float>> &last_path,const std::vector<Pose<float>> &current_path)
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
//    for(int i = 0;i<current_path.size();i++)
//    {
//      if(last_path.at(i+index).heading_angle != current_path.at(i).heading_angle ||
//         last_path.at(i+index).position.x    != current_path.at(i).position.x ||
//         last_path.at(i+index).position.y    != current_path.at(i).position.y)
//      {
//        return true;
//      }
//    }
//    return false;
//  }
//}
//void ComputeControl(const float dt, MPC *mpc, int sock)
//{
//  //1获取机器人当前位置
//  MTX_POSE.lock();
//  Pose<float> cur_pose = CUR_POSE;
//  MTX_POSE.unlock();
//  //2.1 如果前方没有障碍物，规划控制量
//  if(!IS_COLLISION)
//  {
//    //2.1.1 更新控制量
//    static State x0(0.0,0.0,0.0,0.0);
//    MTX_CONTROL.lock();
//    //进行当前状态量的更新
//    x0.v   = actual_linear_velocity;
//    x0.x   = cur_pose.position.x;
//    x0.y   = cur_pose.position.y;
//    x0.yaw = cur_pose.heading_angle;
//    //求解计算
//    //std::cout<<"开始MPC"<<std::endl;
//    mpc_result_.result = mpc->MpcSolve(x0);
//    //std::cout<<"结束MPC"<<std::endl;
//    mpc_result_.result_num++;
//    MTX_CONTROL.unlock();
//    //2.1.2 根据当前状态和控制更新一次CUR_POSE,防止拿不到里程数据

//    //将位置和速度信息都置为0，方便清路并停车
//    x0.x   = mpc_result_.result[default_param_.x_start + 1];
//    x0.y   = mpc_result_.result[default_param_.y_start + 1];
//    x0.yaw = mpc_result_.result[default_param_.yaw_start + 1];
//    x0.v   = mpc_result_.result[default_param_.v_start + 1];
//    //说明当前路径已经结束,不需要进行mpc求解
//    if(x0.x == 0.0 && x0.y == 0.0 && x0.yaw == 0.0 && x0.v == 0.0)
//    {
//      path_size = 0;
//      last_path.clear();
//      std::cout<<"到达终点,路径长度置为0"<<std::endl;
//    }
//  }
//  //2.2 检测到障碍物
//  else
//  {
//    std::cout<<"检测到障碍物,置零速"<<std::endl;
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
//        mpc_result_.result_num = -1;
//        mpc_result_.result.clear();
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
//    std::vector<float> control_data;
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
//static void OnPathUpdate(const std::string &url, MPC *mpc)
//{
//  int sock = create_socket_sub(url);
//  while (1)
//  {
//    std::vector<Pose<float>> path;
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

//          path_size = path.size();
//          float currnet_vel = actual_linear_velocity;
//          mpc->SetTrack(path,currnet_vel);
//          //std::cout<<"收到需要处理的路径,路径长度为:"<<path.size()<<std::endl;
//        }
//        last_path = path;
//        MTX_CONTROL.unlock();
//      }
//      else
//      {
//        //std::cout<<"线路太短"<<std::endl;
//        //path_size = 0;
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

//    Pose<float> odom_pose;
//    if(recv_data(sock, &odom_pose))
//    {
//      //std::cout<<"time_stamp_us="<<time_stamp_us()<<std::endl;
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
//      std::cout<<"障碍物状况："<<IS_COLLISION<<std::endl;
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
//    std::vector<float> control_data;
//    if(recv_data(sock, &control_data))
//    {
//      //MTX_CONTROL.lock();
//      actual_linear_velocity = control_data[1];
//      actual_head_angle      = control_data[2];
//      controller::MessageToRos::GetInstance()->ActualVelPublish(actual_linear_velocity,actual_head_angle);
//      //MTX_CONTROL.unlock();
//    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(1));
//  }
//}
//static void thread_velocity_pub(const std::string &url)
//{
//  //1 定义控制量
//  float velocity = 0.0;
//  float steer_angle = 0.0;
//  int sock = create_socket_pub(url);
//  //int index=0;//
//  int last_mpc_result_num = -1;

//  for(;;)
//  {
//    auto t1 = std::chrono::high_resolution_clock::now();
//    //2 还没有mpc计算结果,发布零速
//    if(mpc_result_.result_num == -1)
//    {
//      velocity = 0.0;
//      steer_angle = 0.0;
//    }
//    else
//    {
//      //3.1 如果前方没有障碍物，规划控制量
//      if(!IS_COLLISION)
//      {
//        if(last_mpc_result_num != mpc_result_.result_num)
//        {
//          last_mpc_result_num = mpc_result_.result_num;
//          //index = 0;
//          //std::cout<<"收到新的mpc计算结果"<<std::endl;
//          continue;
//        }
//        velocity = mpc_result_.result[default_param_.v_start+1];
//        steer_angle = mpc_result_.result[default_param_.delta_start];
//        //速度以及车头转角的平滑
//        mpc::clamp(velocity,(float)(actual_linear_velocity-0.5*0.03),(float)(actual_linear_velocity+0.5*0.03));
//        mpc::clamp(steer_angle,(float)(actual_head_angle - 0.35*0.03),(float)(actual_head_angle + 0.35*0.03));
//        mpc::clamp(steer_angle,(float)(-20.0/180*M_PI),(float)(20.0/180*M_PI));
//      }
//      //3.2 检测到障碍物
//      else
//      {
//        std::cout<<"检测到障碍物,发布零速"<<std::endl;
//        velocity = 0.0;
//        steer_angle = actual_head_angle;
//      }
//    }
//    //4.发布控制量
//    controller::MessageToRos::GetInstance()->ComputeVelPublish(velocity,steer_angle);
//    std::vector<float> control_data;
//    control_data.resize(3);
//    control_data[0] = 0.03;
//    control_data[1] = velocity;
//    control_data[2] = steer_angle;

//    if(!send_data(sock, control_data))
//    {
//      spdlog::error("send msg error");
//    }
//    auto t2 = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<float> time_span = std::chrono::duration_cast<std::chrono::duration<float>>(t2 - t1);
//    int dt = std::ceil(30 - (time_span.count()*1000));
//    std::this_thread::sleep_for(std::chrono::milliseconds(dt));
//  }
//}
//int main(int argc, char **argv)
//{
//  init_log("tracker_mpc");
//  ros::init(argc,argv,"tracker_mpc");
//  std::cout<<"运动控制器正常启动"<<std::endl;
//  last_path.clear();

//  CUR_POSE.position.x    = FLT_MAX;
//  CUR_POSE.position.y    = FLT_MAX;
//  CUR_POSE.heading_angle = FLT_MAX;

//  mpc_result_.result_num = -1;
//  mpc_result_.result.clear();

//  if(!import_nng_configs("/home/caopan/bz_robot/params/nano_msg.json", &NNG_URLS))
//  {
//      exit(-1);
//  }
//  MPC mpc;
//  //std::thread传入参数（）,第1个参数为函数名，后面要传入函数的参数
//  std::vector<std::thread*> thread_list;
//  thread_list.push_back(new std::thread(OnOdomUpdate, NNG_URLS[NNG_ID_NAV_ODOM_SUB]));
//  thread_list.push_back(new std::thread(OnPathUpdate, NNG_URLS[NNG_ID_NAV_LOCAL_PLAN_SUB], &mpc));
//  thread_list.push_back(new std::thread(thread_velocity_pub,NNG_URLS[NNG_ID_ROBOT_TRACK_VELOCITY_PUB]));
//  thread_list.push_back(new std::thread(VelocityControlThread, &mpc, NNG_URLS[NNG_URL_ROBOT_MPC_COMPUTE_PUB]));
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
