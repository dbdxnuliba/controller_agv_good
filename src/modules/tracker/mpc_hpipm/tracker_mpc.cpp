#include "tracker_mpc.h"
#include "common/periodic_task.h"
namespace bz_robot
{

tracker_mpc::tracker_mpc()
{
  std::cout<<"开始MPC控制器初始化"<<std::endl;
  p_mpc_ = std::make_shared<MPC>();
  Init();

  goal_.data.position.x               = 0.0;
  goal_.data.position.y               = 0.0;
  goal_.data.heading_angle            = 0.0;

  robot_current_pose_.data.position.x = 0.0;
  robot_current_pose_.data.position.y = 0.0;
  robot_current_pose_.data.heading_angle = 0.0;

  is_compute_mpc_ = true;

  mpc_result_.result_num = -1;
  mpc_result_.result.clear();
}

tracker_mpc::~tracker_mpc()
{

}

bool tracker_mpc::import_config(const char *file_path)
{
  try
  {
      std::ifstream i(file_path);

      if(i)
      {
          nlohmann::json j;
          i >> j;
          return true;
      }
      else
      {
          printf("can't read config files from: %s\n", file_path);
      }
  }
  catch( std::exception& e )
  {
      printf("%s\n", e.what());
  }
  catch(...)
  {
      printf("un expexted occured\n");
  }
  return false;
}
void tracker_mpc::ComputeMpc()
{
  while (true)
  {
    PERIODIC_MS_TASK(DT);
    //1.循环结束的判断条件
    if(!is_compute_mpc_)
    {
      mpc_result_.result.clear();
      mpc_result_.result_num = -1;
      break;
    }
    //2.计算主循环
    Pose<FLOAT_T> currnet_pose = robot_current_pose_.data;
    ControlData   current_vel  = robot_current_vel_.data;
    Msg<Pose<FLOAT_T> > goal   = goal_;
    mpc_result_.result = p_mpc_->MpcSolve(currnet_pose,current_vel,goal);
    mpc_result_.result_num++;
  }
}
bool tracker_mpc::set_path(const Msg<PathData> &path, const Msg<ControlData> &msg_cur_control_data)
{
  if(path.data.size()<=1)
  {
    std::cout<<"路径太短，不做处理"<<std::endl;
    return true;
  }
  if(!IsNeedSetPath(last_path_,path))
  {
    std::cout<<"保持上次的路径，不做处理"<<std::endl;
    return true;
  }
  reached_target_pose_ = false;
  p_mpc_->SetTrack(path.data, msg_cur_control_data.data);
  last_path_ = path;
  return true;
}

bool tracker_mpc::set_goal(const Msg<Pose<FLOAT_T> > &goal)
{
  goal_ = goal;
  //说明收到新的规划任务，开启一个线程用来进行mpc的计算
  is_compute_mpc_ = true;
  std::thread t(&tracker_mpc::ComputeMpc,this);
  t.detach();
  return true;
}

Msg<ControlData> tracker_mpc::scroll_calculate(const FLOAT_T &dt, const Msg<Pose<FLOAT_T> > &msg_location, const Msg<ControlData> &msg_cur_control_data)
{
  Msg<ControlData> msg;
  msg.time_stamp_us = time_stamp_us();
  msg.data.dt = dt;
  robot_current_pose_ = msg_location;
  robot_current_vel_  = msg_cur_control_data;
  if(reached_target_pose_ || mpc_result_.result_num == -1)
  {
    is_compute_mpc_ = false;
    msg.data.steer_angle = 0.0;
    msg.data.velocity    = 0.0;
    return msg;
  }
  ControlData control_param = SetRobotVelocity(dt);
  msg.data = std::move(control_param);
  //如果收到零速，将参数初始化
  if(control_param.steer_angle==0.0&&control_param.velocity==0.0)
  {
    Init();
    std::cout<<"下发零速"<<std::endl;
  }
  return msg;
}

bool tracker_mpc::IsNeedSetPath(const Msg<PathData> &last_path, const Msg<PathData> &current_path)
{
  int current_size = current_path.data.size();
  int last_size    = last_path.data.size();
  //1.说明第一次收到路径
  if(last_size == 0)
  {
    return true;
  }
  //2.从新路的终点反向遍历，与老路进行对比，如果存在不相同的点，就需要更新路径
  //2.1如果index<0,说明路径在变长，需要更新
  if(last_size-current_size < 0)
  {
    return true;
  }
  //2.2
  for(int i = 1;i<=current_path.data.size();i++)
  {
    if(last_path.data.at(last_size-i).heading_angle != current_path.data.at(current_size-i).heading_angle ||
       last_path.data.at(last_size-i).position.x    != current_path.data.at(current_size-i).position.x ||
       last_path.data.at(last_size-i).position.y    != current_path.data.at(current_size-i).position.y)
    {
      return true;
    }
  }
  return false;
}

ControlData tracker_mpc::SetRobotVelocity(const float &dt)
{
  ControlData msg;
  float target_delta = mpc_result_.result[default_param_.delta_start];
  float target_vx    = mpc_result_.result[default_param_.v_start+1]/std::sqrt(1 + 0.25 * std::pow(std::tan(target_delta),2));
  //速度以及车头转角的平滑
  clamp(target_vx,robot_current_vel_.data.velocity-default_param_.max_accel*dt,
             robot_current_vel_.data.velocity+default_param_.max_accel*dt);
  clamp(target_delta,robot_current_vel_.data.steer_angle - default_param_.max_steer_angle*dt,
             robot_current_vel_.data.steer_angle + default_param_.max_steer_angle*dt);
  clamp(target_delta,-default_param_.max_steer_angle,default_param_.max_steer_angle);

  msg.velocity    = target_vx;
  msg.steer_angle = target_delta;

  return msg;
}

void tracker_mpc::Init()
{
  last_path_.data.clear();
  reached_target_pose_     = true;
}

}
