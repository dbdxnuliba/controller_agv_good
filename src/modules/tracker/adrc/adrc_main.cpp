#if 1
// C HEADER
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <math.h>
#include <time.h>
// C++ HEADER
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <exception>
#include <sstream>
#include <atomic>
// LINUX C HEADER
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <sys/timerfd.h>
#include <signal.h>
//ROS HEADER

// CUSTOM HEADER
#include "common/spdlog/spdlog.h"
#include "common/time_keeper.h"
#include "common/common.h"
#include "common/print.h"
#include "common/periodic_task.h"
#include "modules/msg/nng.h"

#include "adrc.h"
#include "generate_random_number.h"
using namespace std;
using namespace adrc;
static std::atomic<bool> IS_COLLISION(false);
static std::recursive_mutex MTX_CONTROL;
static std::recursive_mutex MTX_POSE;
static Pose<float> CUR_POSE;

static float actual_linear_velocity = 0.0;
static float actual_head_angle = 0.0;

static std::vector<Pose<float>> last_path;
std::map<NNG_ID, std::string> NNG_URLS;

static float kControlCycle = 0.03;//控制周期为30ms
static std::atomic<int> path_size(0);
static std::atomic<bool> odom_update(false);
bool IsNeedSetPath(const std::vector<Pose<float>> &last_path,const std::vector<Pose<float>> &current_path)
{
  int index = last_path.size() - current_path.size();
  //说明第一次收到路径
  if(last_path.size() == 0)
  {
    return true;
  }
  //如果终点不相同,需要设置路径
  if(last_path.at(last_path.size()-1).heading_angle != current_path.at(current_path.size()-1).heading_angle ||
     last_path.at(last_path.size()-1).position.x    != current_path.at(current_path.size()-1).position.x ||
     last_path.at(last_path.size()-1).position.y    != current_path.at(current_path.size()-1).position.y)
  {
    return true;
  }
  //如果终点相同,但index<0,说明路径一定有变化
  if(index<0)
  {
    return true;
  }
  else
  {
    for(int i = 0;i<current_path.size();i++)
    {
      if(last_path.at(i+index).heading_angle != current_path.at(i).heading_angle ||
         last_path.at(i+index).position.x    != current_path.at(i).position.x ||
         last_path.at(i+index).position.y    != current_path.at(i).position.y)
      {
        return true;
      }
    }
    return false;
  }
}
void FromBase(const float &vs,const float &delta,float &v)
{
  v = vs * std::sqrt(1 + 0.25 * std::pow(std::tan(delta),2));
}
void ToBase(const float &v, const float &u0,float &vs, float &delta)
{
  delta = u0;
  vs    = v / std::sqrt(1 + 0.25 * std::pow(std::tan(delta),2));
}
void ComputeControl(const float dt, adrc_controller *ad, int sock)
{
  //1获取机器人当前位置
  MTX_POSE.lock();
  Pose<float> cur_pose = CUR_POSE;
  odom_update = false;
  MTX_POSE.unlock();
  //2 定义控制量
  float velocity = 0.0;
  float steer_angle = 0.0;
  //2.1 如果前方没有障碍物，规划控制量
  if(!IS_COLLISION)
  {
    //2.1.1 更新控制量
    static State x0(0.0,0.0,0.0,0.0);
    MTX_CONTROL.lock();
    //进行当前状态量的更新
    FromBase(actual_linear_velocity,actual_head_angle,x0.v);
    x0.x   = cur_pose.position.x;
    x0.y   = cur_pose.position.y;
    x0.yaw = cur_pose.heading_angle;

    VectorX2<float> fuzzy_resut = ad->RunAdrc(x0,actual_head_angle);
    ToBase(fuzzy_resut.x,fuzzy_resut.y,velocity,steer_angle);
    if(!odom_update)
    {
      CUR_POSE.position.x    = x0.x + x0.v * std::cos(x0.yaw) * kControlCycle;
      CUR_POSE.position.y    = x0.y + x0.v * std::sin(x0.yaw) * kControlCycle;
      CUR_POSE.heading_angle = x0.yaw + x0.v/0.64*std::tan(actual_head_angle)*kControlCycle;
    }

    MTX_CONTROL.unlock();
    //说明当前路径已经结束,不需要进行mpc求解
    if(velocity == 0 && steer_angle == 0)
    {
      path_size = 0;
      last_path.clear();
      std::cout<<"到达终点,路径长度置为0"<<std::endl;
    }
  }
  //2.2 检测到障碍物
  else
  {
    PRINT_ERROR("detect collision");
  }
  mpc::clamp(velocity,(float)(actual_linear_velocity-0.5*0.03),(float)(actual_linear_velocity+0.5*0.03));
  mpc::clamp(steer_angle,(float)(actual_head_angle - 0.349*0.03),(float)(actual_head_angle + 0.349*0.03));
  actual_linear_velocity = velocity;
  actual_head_angle      = steer_angle;
  //3.发布控制量
  std::vector<float> control_data;
  control_data.resize(3);
  control_data[0] = dt;
  control_data[1] = velocity;
  control_data[2] = steer_angle;
  controller::MessageToRos::GetInstance()->ActualVelPublish(velocity,steer_angle);
  if(!send_data(sock, control_data))
  {
    spdlog::error("send msg error");
  }
}
void VelocityControlThread(adrc_controller *ad, const string & url)
{
  int sock = create_socket_pub(url);
  try
  {
    while(1)
    {
      if(path_size == 0)
      {
        //std::cout<<"fuzzy控制器未收到路径信息,不需要执行计算控制量的线程"<<std::endl;
        std::vector<float> control_data;
        control_data.resize(3);
        control_data[0] = 0.0;
        control_data[1] = 0.0;
        control_data[2] = 0.0;
        if(!send_data(sock, control_data))
        {
          spdlog::error("send msg error");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
      else
      {
        PERIODIC_TASK(kControlCycle * 1000);//TASK_PERIOD_MS = 30ms
        //RECORD_TIME();
        ComputeControl(kControlCycle, ad, sock);
      }
    }
  }
  catch(...)
  {
    std::vector<float> control_data;
    control_data.resize(3);
    control_data[0] = 0.0;
    control_data[1] = 0.0;
    control_data[2] = 0.0;
    if(!send_data(sock, control_data))
    {
      spdlog::error("send msg error");
    }
  }
}
static void OnPathUpdate(const std::string &url, adrc_controller *ad)
{
  int sock = create_socket_sub(url);
  while (1)
  {
    std::vector<Pose<float>> path;
    if(recv_data(sock, &path))
    {
      if(path.size()>1)
      {
        MTX_CONTROL.lock();
        if(!IsNeedSetPath(last_path,path))
        {
          //std::cout<<"收到不需要处理的路径"<<std::endl;
        }
        else
        {

          path_size = path.size();
          float currnet_vel = actual_linear_velocity;
          ad->SetTrack(path,currnet_vel);
          //std::cout<<"收到需要处理的路径,路径长度为:"<<path.size()<<std::endl;
        }
        last_path = path;
        MTX_CONTROL.unlock();
      }
      else
      {
        //std::cout<<"线路太短"<<std::endl;
        path_size = 0;
        ad->ClearTrack();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
static void OnOdomUpdate(const std::string &url)
{
  int sock = create_socket_sub(url);
  for (;;)
  {
    auto t1 = std::chrono::high_resolution_clock::now();
    NanoMsg<Pose<float>> msg_odom;
    Pose<float> &odom_pose = msg_odom.data;
    if(recv_data(sock, &msg_odom))
    {
      MTX_POSE.lock();
      CUR_POSE = odom_pose;
      odom_update = true;
      //CUR_POSE.position.x = controller::GenerateRandom::GetInstance()->GenerateGaussian(odom_pose.position.x,0.05);
      //CUR_POSE.position.y = controller::GenerateRandom::GetInstance()->GenerateGaussian(odom_pose.position.y,0.05);
      //CUR_POSE.heading_angle = controller::GenerateRandom::GetInstance()->GenerateGaussian(odom_pose.heading_angle,0.05);

      controller::MessageToRos::GetInstance()->RobotPosePublish(CUR_POSE.position.x,CUR_POSE.position.y,
                                                          CUR_POSE.heading_angle);
      MTX_POSE.unlock();
    }
    //if(msg_odom.time_stamp_us - msg_odom.time_stamp_us > 300 * 1000)
    {
//      PRINT_DEBUG("odom {:.3f},{:.3f}", odom_pose.position.x, odom_pose.position.y);
//    PRINT_DEBUG("odom time{:.6f}\t time now{:.6f}", 0.001 * 0.001 * msg_odom.time_stamp_us,
//                0.001 * 0.001 * time_stamp_us());
    }
    //auto t2 = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<float> time_span = std::chrono::duration_cast<std::chrono::duration<float>>(t2 - t1);
    //std::cout<<"delta_time="<<time_span.count()<<std::endl;
    //里程计消息有抖动,不要过于相信,后续考虑添加滤波处理模块,因此这边改为每100ms获取一次
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

static void OnCollisionDetection(const std::string &url)
{
  int sock = create_socket_sub(url);
  while (1)
  {
    bool is_collision;
    if(recv_data(sock, &is_collision))
    {
      IS_COLLISION = is_collision;
    }
    else
    {
      IS_COLLISION = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
static void thread_sub_actual_velocity(const std::string &url)
{
  int sock = create_socket_sub(url);
  for (;;)
  {
    std::vector<float> control_data;
    if(recv_data(sock, &control_data))
    {
      actual_linear_velocity = control_data[1];
      actual_head_angle      = control_data[2];
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
int main(int argc, char **argv)
{
  init_log("tracker_adrc");
  ros::init(argc,argv,"tracker_adrc");
  std::cout<<"运动控制器正常启动"<<std::endl;
  last_path.clear();

  CUR_POSE.position.x    = FLT_MAX;
  CUR_POSE.position.y    = FLT_MAX;
  CUR_POSE.heading_angle = FLT_MAX;

  if(!import_nng_configs("/home/caopan/bz_robot/params/nano_msg.json", &NNG_URLS))
  {
      exit(-1);
  }
  adrc_controller ad;
  //std::thread传入参数（）,第1个参数为函数名，后面要传入函数的参数
  std::vector<std::thread*> thread_list;
  thread_list.push_back(new std::thread(OnOdomUpdate, NNG_URLS[NNG_ID_NAV_ODOM_SUB]));
  thread_list.push_back(new std::thread(VelocityControlThread, &ad,NNG_URLS[NNG_ID_ROBOT_TRACK_VELOCITY_PUB]));
  thread_list.push_back(new std::thread(OnPathUpdate, NNG_URLS[NNG_ID_NAV_LOCAL_PLAN_SUB], &ad));
  thread_list.push_back(new std::thread(OnCollisionDetection, NNG_URLS[NNG_ID_NAV_COLLISION_DETECTION_SUB]));
  thread_list.push_back(new std::thread(thread_sub_actual_velocity, NNG_URLS[NNG_ID_ROBOT_ACTUAL_VELOCITY_SUB]));
  //detach实现子线程与主线程的分离，
  for(auto it = thread_list.begin(); it != thread_list.end(); ++it)
  {
    (*it)->detach();
  }
  //通过循环保持进程，使得thread_list的线程正常运行
  while (1)
  {
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
  std::cout<<"运动控制器正常结束"<<std::endl;
  return 0;
}
#endif
