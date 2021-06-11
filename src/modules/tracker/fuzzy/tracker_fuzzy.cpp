#include "tracker_fuzzy.h"
namespace bz_robot
{
tracker_fuzzy::tracker_fuzzy()
{
  std::cout<<"开始模糊控制器初始化"<<std::endl;
  p_fuzzy_                 = std::make_shared<FuzzyPID>();
  Init();
  goal_.data.position.x    = 0.0;
  goal_.data.position.y    = 0.0;
  goal_.data.heading_angle = 0.0;

  is_compute_fuzzy_ = false;
}

tracker_fuzzy::~tracker_fuzzy()
{

}

bool tracker_fuzzy::import_config(const char *file_path)
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

bool tracker_fuzzy::set_path(const Msg<PathData> &path, const Msg<ControlData> &msg_cur_control_data)
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
  is_compute_fuzzy_    = true;
  p_fuzzy_->SetTrack(path.data, msg_cur_control_data.data);
  last_path_ = path;
  return true;
}

bool tracker_fuzzy::set_goal(const Msg<Pose<FLOAT_T> > &goal)
{
  goal_ = goal;
  return true;
}

Msg<bool> tracker_fuzzy::is_goal_reached()
{
  Msg<bool> msg;
  msg.data = reached_target_pose_;
  //std::cout<<"reached_target_pose_="<<reached_target_pose_<<std::endl;
  return msg;
}

bool tracker_fuzzy::set_stop_signal(const Msg<bool> &stop)
{
  is_compute_fuzzy_ = !stop.data;
  //std::cout<<"收到停止控制器的信号"<<std::endl;
  return true;
}

Msg<ControlData> tracker_fuzzy::scroll_calculate(const FLOAT_T &dt, const Msg<Pose<FLOAT_T> > &msg_location, const Msg<bz_robot::ControlData> &msg_cur_control_data)
{
  Msg<ControlData> msg;
  msg.time_stamp_us = time_stamp_us();
  msg.data.dt = dt;
  if(reached_target_pose_ || !is_compute_fuzzy_)
  {
    msg.data.steer_angle = 0.0;
    msg.data.velocity    = 0.0;
    std::cout<<"reached_target_pose_:"<<reached_target_pose_<<"is_compute_fuzzy_:"<<is_compute_fuzzy_<<std::endl;
    return msg;
  }
  ControlData control_param = p_fuzzy_->RunFuzzy(msg_location.data,msg_cur_control_data.data,goal_);
  msg.data = std::move(control_param);
  //如果收到零速，将参数初始化
  if(control_param.steer_angle==0.0&&control_param.velocity==0.0)
  {
    Init();
    std::cout<<"下发零速"<<std::endl;
  }
  return msg;
}

bool tracker_fuzzy::IsNeedSetPath(const Msg<PathData> &last_path, const Msg<PathData> &current_path)
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
  for(uint i = 1;i<=current_path.data.size();i++)
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

void tracker_fuzzy::Init()
{
  last_path_.data.clear();
  reached_target_pose_     = true;
}
}

