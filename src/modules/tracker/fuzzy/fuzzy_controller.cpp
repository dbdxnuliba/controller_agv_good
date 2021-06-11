#include "fuzzy_controller.h"
namespace bz_robot {

FuzzyPID::FuzzyPID()
{
  frenet_hdr_ = std::make_shared<Frenet>();
  engine_           = new Engine;
  input_error_      = new InputVariable;
  input_error_rate_ = new InputVariable;
  output_delta_     = new OutputVariable;
  mamdani_          = new RuleBlock;
  Init();
  last_forward_delta_=0.0;
}

FuzzyPID::~FuzzyPID()
{
  delete engine_ ;
  delete input_error_   ;
  delete input_error_rate_ ;
  delete output_delta_     ;
  delete mamdani_          ;
}

void FuzzyPID::SetTrack(const PathData &path, const ControlData &data)
{
  //转化到机器人中心
  float velocity = FromBase(data.velocity,data.steer_angle);
  track_.DealWithPath(path,velocity);
}

void FuzzyPID::ClearTrack()
{
  track_.ClearSpline();
}

std::vector<Pose<FLOAT_T> > FuzzyPID::GetTrackNew()
{
  std::vector<Pose<FLOAT_T> > paths;
  Pose<FLOAT_T> path(0.0,0.0,0.0);
  paths.clear();
  SplineParam spline = track_.GetTrack();
  for(int i=0;i<spline.x.size();i++)
  {
    path.position.x = spline.x.at(i);
    path.position.y = spline.y.at(i);
    paths.push_back(path);
  }
  return paths;
}

ControlData FuzzyPID::RunFuzzy(const Pose<FLOAT_T> &state, const ControlData &current_vel, Msg<Pose<FLOAT_T> > &goal)
{
  //1.如果判断机器人已经超过目标位置，则需要返回零值
  ControlData control = current_vel;
  State x(0.0,0.0,0.0,0.0);
  //转化到机器人中心
  control.velocity = FromBase(control.velocity,control.steer_angle);
  x.x   = state.position.x;
  x.y   = state.position.y;
  x.yaw = state.heading_angle;
  x.v   = control.velocity;
  bz_robot::MessageToRos::GetInstance()->RobotPosePublish(state.position.x,state.position.y,state.heading_angle);
  if(track_.IsGoReached(x))
  {
    //std::cout<<"机器人位置x:"<<x.x<<"机器人位置y:"<<x.y<<"机器人位置yaw:"<<x.yaw<<std::endl;
    //std::cout<<"终点位置x:"<<track_.GetTrack().x.back()<<"机器人位置y:"<<track_.GetTrack().y.back()<<"机器人位置yaw:"<<track_.GetTrack().yaw.back()<<std::endl;
    control.velocity    = 0.0;
    control.steer_angle = 0.0;
    control.dt          = param_.dt;
    last_forward_delta_=0.0;
    //ClearTrack();
    return control;
  }
  //std::cout<<"==================================="<<std::endl;
  //2.如果没有到达终点，进行偏差的计算,方法1是转化frenet坐标系，计算精确，但是计算量偏大，方法2精度低，但是计算量小
  //2.1首先获取到小车到线路的垂足距离，以及最近点编号
  float robot_x   = x.x;
  float robot_y   = x.y;
  float robot_yaw = x.yaw;
  SplineParam spline = track_.GetTrack();
  std::vector<float> frenet_return = frenet_hdr_->ToFrenet(robot_x,robot_y,robot_yaw,
                                                           spline.x,spline.y);
  //todo 可视化三次样条插值后的路径
  std::vector<geometry_msgs::Point> points;
  for(int i=0;i<spline.x.size();i++)
  {
    geometry_msgs::Point p;
    p.x = spline.x.at(i);
    p.y = spline.y.at(i);
    p.z = 0.0;
    points.push_back(p);
  }
  bz_robot::MessageToRos::GetInstance()->VisuallocalPath(points);
  //2.2计算横向偏差以及角度偏差
  //计算预瞄点的前馈倍率，基数为向前十个点，倍数在1-10之间，倍数与机器人线速度以及局部路径的曲率有关
  float lookahead_ratio = 5.0;
  lookahead_ratio = lookahead_ratio*std::max(current_vel.velocity,(float)0.6);
//  lookahead_ratio = 0.1*lookahead_ratio/spline.curvature.at(frenet_return[0]);
//  lookahead_ratio = Clamp(lookahead_ratio,(float)2.0,(float)5.0);
  float lateral_error = frenet_return[1];
  int point1          = frenet_return[0] + 30;
  int point           = frenet_return[0] + 10*lookahead_ratio;
  int max_point       = spline.yaw.size()-1;
  point1              = Clamp(point1,0,max_point);
  //point2 = Clamp(point2,0,max_point);
  point               = Clamp(point,0,max_point);
  float yaw_error     = angles::normalize_angle(robot_yaw - spline.yaw[point]);
  //controller::MessageToRos::GetInstance()->SplineParamPublish(robot_yaw,spline.yaw[point]);
  bz_robot::MessageToRos::GetInstance()->VisualTargetPoint(spline.x[point],spline.y[point],spline.yaw[point]);
  bz_robot::MessageToRos::GetInstance()->VisualFootPoint(spline.x[frenet_return[0]+10],spline.y[frenet_return[0]+10],spline.yaw[frenet_return[0]+10]);
  //todo如果是倒车，需要将角度偏差进行取反,判断依据是什么，是接受路径时对路径速度信息进行赋值？？？？
  //2.3计算混合偏差
  float current_error = SynthesizedError(lateral_error,yaw_error);
  //2.4计算偏差变化率
  float d_error = ComputeErrorRate(control.steer_angle,yaw_error,x.v);
  //4.进行具体的模糊计算，包括输入模糊化，模糊推理，解模糊等过程
  control.velocity    = spline.allowed_speed.at(frenet_return[0]);//速度
  control.steer_angle = RealizeFuzzy(current_error,d_error);//偏向角
  //当发现需要过小半径圆弧时
  int point2          = frenet_return[0] + 60;
  if(point2 <= max_point)
  {
    //计算1m长度下的偏向角变化
    float delta_yaw = std::fabs(angles::normalize_angle(spline.yaw.at(frenet_return[0]+10) - spline.yaw.at(point2)));
    if(delta_yaw>0.2 && current_vel.velocity>=0.3)
    {
      //std::cout<<"降速通过,角度偏差为："<<delta_yaw<<std::endl;
      //std::cout<<"降速前速度为："<<control.velocity<<"/车头偏向角为："<<control.steer_angle<<std::endl;
      control.velocity = std::max(0.3,(-4*std::pow(delta_yaw,2)+1.16));
      control.steer_angle = control.steer_angle*std::min(2.0,6*std::pow(delta_yaw,2)+0.76);
      //std::cout<<"降速后速度为："<<control.velocity<<"/车头偏向角为："<<control.steer_angle<<std::endl;
    }
  }

  //std::cout<<"模糊计算速度="<<control.velocity<<std::endl;
  bz_robot::MessageToRos::GetInstance()->ErrorStatusPublish(lateral_error,yaw_error,current_error,d_error,(float)control.steer_angle);
  //过小圆弧
//  if(std::fabs(angles::normalize_angle(spline.yaw4[frenet_return[0]] - spline.yaw[point]))>0.15)
//  {
//    control.steer_angle = control.steer_angle - 0.5*angles::normalize_angle(spline.yaw[frenet_return[0]] - spline.yaw[point]);
//  }
  if(point1 == max_point)
  {
    control.steer_angle = control.steer_angle - 0.8*angles::normalize_angle(robot_yaw - goal.data.heading_angle);
  }
  clamp(control.steer_angle,-param_.max_steer_angle,param_.max_steer_angle);

  clamp(control.velocity,param_.initial_vel,param_.max_vel);

  clamp(control.velocity,current_vel.velocity-param_.max_acc*param_.dt,current_vel.velocity+param_.max_acc*param_.dt);
  clamp(control.steer_angle,current_vel.steer_angle-param_.max_steer_angle*param_.dt,current_vel.steer_angle+param_.max_steer_angle*param_.dt);
  //转化到机器人后轮中心
  control.velocity = ToBase(control.velocity,control.steer_angle);
  bz_robot::MessageToRos::GetInstance()->ComputeVelPublish(control.velocity,control.steer_angle);
  return control;
}

void FuzzyPID::Init()
{
  engine_->setName("test");
  engine_->setDescription("");
  //定义输入误差的隶属度函数关系：Triangle代表三角隶属度，Trapezoid代表梯形隶属度
  input_error_->setEnabled(true);
  input_error_->setName("e");
  input_error_->setRange(-0.54, 0.54);
  input_error_->setLockValueInRange(true);
  input_error_->addTerm(new fl::Triangle("NB1", -0.54, -0.54, -0.45));
  input_error_->addTerm(new fl::Triangle("NB0", -0.54, -0.45, -0.36));
  input_error_->addTerm(new fl::Triangle("NM1", -0.45, -0.36, -0.27));
  input_error_->addTerm(new fl::Triangle("NM0", -0.36, -0.27, -0.18));
  input_error_->addTerm(new fl::Triangle("NS1", -0.27, -0.18, -0.09));
  input_error_->addTerm(new fl::Triangle("NS0", -0.18, -0.06, 0.0));
  input_error_->addTerm(new fl::Triangle("ZO", -0.06, 0.0, 0.06));
  input_error_->addTerm(new fl::Triangle("PS0", 0.0, 0.06, 0.18));
  input_error_->addTerm(new fl::Triangle("PS1", 0.09, 0.18, 0.27));
  input_error_->addTerm(new fl::Triangle("PM0", 0.18, 0.27, 0.36));
  input_error_->addTerm(new fl::Triangle("PM1", 0.27, 0.36, 0.45));
  input_error_->addTerm(new fl::Triangle("PB0", 0.36, 0.45, 0.54));
  input_error_->addTerm(new fl::Triangle("PB1", 0.45, 0.54, 0.54));

  engine_->addInputVariable(input_error_);

  //定义输入误差变化率的隶属度函数关系
  input_error_rate_->setEnabled(true);
  input_error_rate_->setName("ce");
  input_error_rate_->setRange(-0.42, 0.42);
  input_error_rate_->setLockValueInRange(true);
  input_error_rate_->addTerm(new fl::Triangle("NB1", -0.42, -0.42, -0.35));
  input_error_rate_->addTerm(new fl::Triangle("NB0", -0.42, -0.35, -0.28));
  input_error_rate_->addTerm(new fl::Triangle("NM1", -0.35, -0.28, -0.21));
  input_error_rate_->addTerm(new fl::Triangle("NM0", -0.28, -0.21, -0.14));
  input_error_rate_->addTerm(new fl::Triangle("NS1", -0.21, -0.14, -0.07));
  input_error_rate_->addTerm(new fl::Triangle("NS0", -0.14, -0.05, 0.0));
  input_error_rate_->addTerm(new fl::Triangle("ZO", -0.05, 0.0, 0.05));
  input_error_rate_->addTerm(new fl::Triangle("PS0", 0.0, 0.05, 0.14));
  input_error_rate_->addTerm(new fl::Triangle("PS1", 0.07, 0.14, 0.21));
  input_error_rate_->addTerm(new fl::Triangle("PM0", 0.14, 0.21, 0.28));
  input_error_rate_->addTerm(new fl::Triangle("PM1", 0.21, 0.28, 0.35));
  input_error_rate_->addTerm(new fl::Triangle("PB0", 0.28, 0.35, 0.42));
  input_error_rate_->addTerm(new fl::Triangle("PB1", 0.35, 0.42, 0.42));

  engine_->addInputVariable(input_error_rate_);

  //定义输出的隶属度函数关系
  output_delta_->setEnabled(true);
  output_delta_->setName("out");
  output_delta_->setDescription("");
  output_delta_->setRange(-0.42, 0.42);
  output_delta_->setLockValueInRange(true);
  output_delta_->setAggregation(new Maximum);
  output_delta_->setLockPreviousValue(true);
  output_delta_->setDefuzzifier(new Centroid(200));
  output_delta_->setDefaultValue(fl::nan);
  output_delta_->addTerm(new fl::Triangle("NB1", -0.42, -0.42, -0.35));
  output_delta_->addTerm(new fl::Triangle("NB0", -0.42, -0.35, -0.28));
  output_delta_->addTerm(new fl::Triangle("NM1", -0.35, -0.28, -0.21));
  output_delta_->addTerm(new fl::Triangle("NM0", -0.28, -0.21, -0.14));
  output_delta_->addTerm(new fl::Triangle("NS1", -0.21, -0.14, -0.07));
  output_delta_->addTerm(new fl::Triangle("NS0", -0.14, -0.05, 0.0));
  output_delta_->addTerm(new fl::Triangle("ZO", -0.05, 0.0, 0.05));
  output_delta_->addTerm(new fl::Triangle("PS0", 0.0, 0.05, 0.14));
  output_delta_->addTerm(new fl::Triangle("PS1", 0.07, 0.14, 0.21));
  output_delta_->addTerm(new fl::Triangle("PM0", 0.14, 0.21, 0.28));
  output_delta_->addTerm(new fl::Triangle("PM1", 0.21, 0.28, 0.35));
  output_delta_->addTerm(new fl::Triangle("PB0", 0.28, 0.35, 0.42));
  output_delta_->addTerm(new fl::Triangle("PB1", 0.35, 0.42, 0.42));

  engine_->addOutputVariable(output_delta_);

  //Rules
  mamdani_->setEnabled(true);
  mamdani_->setName("mamdani");
  mamdani_->setDescription("Mamdani inference");
  mamdani_->setConjunction(new fl::Minimum);
  mamdani_->setDisjunction(new fl::Maximum);
  mamdani_->setImplication(new AlgebraicProduct);
  mamdani_->setActivation(new General);
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is NB1 then out is PB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is NB0 then out is PB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is NM1 then out is PB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is NM0 then out is PB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is NS1 then out is PB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is NS0 then out is PB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is ZO then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is PS0 then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is PS1 then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is PM0 then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is PM1 then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is PB0 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB1 and ce is PB1 then out is PM0", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is NB1 then out is PB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is NB0 then out is PB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is NM1 then out is PB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is NM0 then out is PB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is NS1 then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is NS0 then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is ZO then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is PS0 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is PS1 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is PM0 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is PM1 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is PB0 then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NB0 and ce is PB1 then out is PS1", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is NB1 then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is NB0 then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is NM1 then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is NM0 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is NS1 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is NS0 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is ZO then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is PS0 then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is PS1 then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is PM0 then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is PM1 then out is PS0 ", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is PB0 then out is ZO ", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM1 and ce is PB1 then out is ZO ", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is NB1 then out is PB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is NB0 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is NM1 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is NM0 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is NS1 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is NS0 then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is ZO then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is PS0 then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is PS1 then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is PM0 then out is ZO ", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is PM1 then out is ZO ", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is PB0 then out is NS0 ", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NM0 and ce is PB1 then out is NS0 ", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is NB1 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is NB0 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is NM1 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is NM0 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is NS1 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is NS0 then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is ZO then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is PS0 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is PS1 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is PM0 then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is PM1 then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is PB0 then out is NS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS1 and ce is PB1 then out is NS1", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is NB1 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is NB0 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is NM1 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is NM0 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is NS1 then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is NS0 then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is ZO then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is PS0 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is PS1 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is PM0 then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is PM1 then out is NS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is PB0 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is NS0 and ce is PB1 then out is NM1", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is NB1 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is NB0 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is NM1 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is NM0 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is NS1 then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is NS0 then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is ZO then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is PS0 then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is PS1 then out is NS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is PM0 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is PM1 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is PB0 then out is NM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is ZO and ce is PB1 then out is NM1", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is NB1 then out is PM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is NB0 then out is PM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is NM1 then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is NM0 then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is NS1 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is NS0 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is ZO then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is PS0 then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is PS1 then out is NS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is PM0 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is PM1 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is PB0 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS0 and ce is PB1 then out is NM1", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is NB1 then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is NB0 then out is PS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is NM1 then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is NM0 then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is NS1 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is NS0 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is ZO then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is PS0 then out is NS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is PS1 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is PM0 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is PM1 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is PB0 then out is NM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PS1 and ce is PB1 then out is NM1", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is NB1 then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is NB0 then out is PS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is NM1 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is NM0 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is NS1 then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is NS0 then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is ZO then out is NS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is PS0 then out is NS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is PS1 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is PM0 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is PM1 then out is NM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is PB0 then out is NM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM0 and ce is PB1 then out is NB0", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is NB1 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is NB0 then out is ZO", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is NM1 then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is NM0 then out is NS0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is NS1 then out is NS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is NS0 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is ZO then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is PS0 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is PS1 then out is NM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is PM0 then out is NM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is PM1 then out is NB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is PB0 then out is NB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PM1 and ce is PB1 then out is NB0", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is NB1 then out is NS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is NB0 then out is NS1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is NM1 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is NM0 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is NS1 then out is NM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is NS0 then out is NM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is ZO then out is NB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is PS0 then out is NB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is PS1 then out is NB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is PM0 then out is NB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is PM1 then out is NB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is PB0 then out is NB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB0 and ce is PB1 then out is NB1", engine_));

  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is NB1 then out is NM0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is NB0 then out is NM1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is NM1 then out is NB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is NM0 then out is NB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is NS1 then out is NB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is NS0 then out is NB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is ZO then out is NB0", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is PS0 then out is NB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is PS1 then out is NB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is PM0 then out is NB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is PM1 then out is NB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is PB0 then out is NB1", engine_));
  mamdani_->addRule(fl::Rule::parse("if e is PB1 and ce is PB1 then out is NB1", engine_));
  engine_->addRuleBlock(mamdani_);

}

float FuzzyPID::SynthesizedError(float &lateral_error, float &angle_error)
{
  float error = 0.0;
  //todo后期是否考虑更合理的融合公式
  error         = 0.35*lateral_error + (1.0-0.35)*angle_error;
  error = error/0.5;
  //std::cout<<"混合偏差："<<error<<std::endl;
  //error         = Clamp(error,-param_.error_max,param_.error_max);
  return error;
}
float FuzzyPID::RealizeFuzzy(const float &error, const float &d_error)
{
  auto t1 = std::chrono::high_resolution_clock::now();
  std::string status;
  if (not engine_->isReady(&status))
  {
    throw Exception("[engine error] engine is not ready:\n" + status, FL_AT);
  }

  //Set inputs
  input_error_->setValue(error);
  input_error_rate_->setValue(d_error);

  //Start fuzzy
  engine_->process();
  scalar out1 = output_delta_->getValue();
  float delta_steer_angle1 = out1;
  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> delta_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  //std::cout<<"利用模糊规则库计算耗时："<<delta_time.count()<<std::endl;
  bz_robot::MessageToRos::GetInstance()->FuzzyIoPublish(error,d_error,0.0,delta_steer_angle1);
  return delta_steer_angle1;
}

float FuzzyPID::ComputeErrorRate(const float &delta, const float &yaw_error, const float &vel)
{
  float y_error_rate;
  float yaw_error_rate;
  float error_rate;
  //1.计算机器人横摆角速度
  float omiga = vel*tan(delta)/param_.wheel_distance;
  //2.计算横向偏差变化率
  y_error_rate = vel*sin(yaw_error-omiga*param_.dt*0.5);
  //3.计算角度偏差变化率
  yaw_error_rate = omiga;
  //4.计算混合偏差变化率
  error_rate = SynthesizedError(y_error_rate,yaw_error_rate);
  //std::cout<<"车头偏向角："<<delta<<"/角速度："<<omiga<<"/横向偏差变化率："<<y_error_rate<<"/角度偏差变化率："<<yaw_error_rate<<std::endl;
  return error_rate;
}
}
