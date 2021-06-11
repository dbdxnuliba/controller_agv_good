/*************************************************************************
  > File Name: cubic_spline.h
  > Author: pan.cao
  > Mail: umrobot
  > Created Time:  8 27 2020
 ************************************************************************/
#pragma once
#include<iostream>
#include<iomanip>
#include<limits>
#include<vector>

#include<sys/time.h>
#include<Eigen/Eigen>
#include<cppad/cppad.hpp>
#include<cppad/ipopt/solve.hpp>
#include<angles/angles.h>

#include "deal_with_track.h"
#include "controller_to_ros.h"

#define DT 0.21
#define SIZE_PREDICTION 12

using M_XREF=Eigen::Matrix<float, 4, SIZE_PREDICTION>;
using CppAD::AD;
using Dvector  = CPPAD_TESTVECTOR(double);

namespace bz_robot {
struct DefaulltParam
{
  int state_number = 4;//状态量个数
  float max_steer_angle = 20.0/180*M_PI;//最大偏转角
  float wheel_distance = 0.64;//轮间距
  float max_speed = 1.0;
  float min_speed = 0.0;
  float max_accel =0.5;

  float initial_velocity = 0.1;

  //各个时刻状态量的编号
  int x_start = 0;
  int y_start = x_start + SIZE_PREDICTION;//12
  int yaw_start = y_start + SIZE_PREDICTION;//24
  int v_start = yaw_start + SIZE_PREDICTION;//36
  //各个时刻控制量的编号,a和delta都只有-1时刻
  int a_start = v_start + SIZE_PREDICTION;//48
  int delta_start = a_start + SIZE_PREDICTION-1;//59
};
static const DefaulltParam default_param_;
struct MpcResult
{
  Vec_f result;
  int   result_num;
};
class MPC
{
public:
  MPC();
  ~MPC();
  /**
     * @brief 求解机器人距离线路最近的点以及个时刻的状态量初始
     * @param 输入参数:当前时刻状态量state,坐标位置x的容器,坐标位置y的容器,坐标yaw角的容器,允许的目标速度容器,
     *                单段路径的长度dl
     * @param 输出参数:本次的最近点target_ind,需要输入求解器的状态量初值矩阵xref
     * @return 返回值:无
     */
  //void CalcRefTrajectory(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f sp, float dl,M_XREF& xref);
  void CalcRefTrajectory(Vec_f &var, Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f sp, float dl,M_XREF& xref);
  /**
     * @brief 根据当前时刻的状态,求解出个时刻的状态和控制量
     * @param 输入参数:当前时刻状态量state
     * @param 输出参数:无
     * @return 返回值:求解出个时刻的状态和控制量
     */
  Vec_f MpcSolve(const Pose<FLOAT_T> &state, const ControlData &current_vel, const Msg<Pose<FLOAT_T> > &goal);
  void SetTrack(const PathData &path, const ControlData &data);
private:
  /**
     * @brief 对机器人状态量进行更新
     * @param 输入参数:当前加速度a,车头偏向角delta
     * @param 输出参数:状态量
     * @return 返回值:无
     */
  void UpdateState(const float &a, float &delta,State &state);
  /**
     * @brief 对y向偏差大于阈值，且会继续增大的情况进行重新的初始化
     * @param 输入参数:当前时刻状态量
     * @param 输出参数:无
     * @return 返回值:无
     */
  void NewInitialInput(const State &x0);
  /**
     * @brief 对新一时刻求解器的输入状态进行更新
     * @param 输入参数:当前时刻状态量
     * @param 输出参数:无
     * @return 返回值:无
     */
  void UpdateInitialInput(const State &x0);
  /**
     * @brief 对初始状态进行异常值处理,如角度正规化,长度正规化等
     * @param 输入参数:无
     * @param 输出参数:无
     * @return 返回值:无
     */
  void SetZero(Vec_f &v);
  /**
     * @brief 将预测的控制量与线路允许的控制量进行混合
     * @param 输入参数:mpc优化结果
     * @param 输出参数:混合后的优化结果
     * @return 返回值:无
     */
  void MixPredictiveAndAllowed(Vec_f &result,State &state);
  /**
     * @brief 对状态量的输入进行处理，保证不超过终点之类的
     * @param 输入参数:无
     * @param 输出参数:无
     * @return 返回值:无
     */
  void WrapInput(const SplineParam &track_param);
  /**
     * @brief 根据输入变量，设置输入变量的上下边界
     * @param 输入参数:输入变量
     * @param 输出参数:输入变量的上下边界
     * @return 返回值:无
     */
  void SetVarBound(const Dvector &var, Dvector &lower_bound, Dvector &upper_bound);
  /**
     * @brief 设置约束的上下边界
     * @param 输入参数:输入变量
     * @param 输出参数:约束的上下边界
     * @return 返回值:无
     */
  void SetConstraintBound(const Dvector &var, Dvector &lower_bound, Dvector &upper_bound);
  /**
     * @brief 设置求解器的参数
     * @param 输入参数:无
     * @param 输出参数:求解器参数
     * @return 返回值:无
     */
  void SetSloverOption(std::string &option);
  /**
     * @brief 获取机器人到轨迹的距离，为减小计算量，该距离只是近似值，机器人离路径越近，误差越大，最大误差1厘米
     * @param 输入参数:机器人当前state
     * @param 输出参数:无
     * @return 返回值:距离
     */
  float DisToPath(const State &x);
  /**
     * @brief 当车辆计算结果不理想的情况下，需要重新初始化输入
     * @param 输入参数:机器人当前state，跟踪路径信息
     * @param 输出参数:车头需要的偏向
     * @return 返回值:是否需要初始化
     */
  bool NeedNewInput(const State &x,const SplineParam &track,float &delta);
  /**
     * @brief 將預測點可視化
     * @param 输入参数:各個時刻的預測狀態量
     * @param 输出参数:无
     * @return 返回值:無
     */
  void VisualPredictionPoint(const Vec_f &var);
  /**
     * @brief 将状态量预测范围控制在管道内
     * @param 输入参数:垂足点处x,y,yaw
     * @param 输出参数:无
     * @return 返回值:无
     */
  void SetRoadBound(const M_XREF &trajectory_point, const Dvector vars,Dvector &lower_bound, Dvector &upper_bound);
  void PrintfSolutionStatus(const int &status);
  float inline FromBase(const float &vs,const float &delta)
  {
   return vs * std::sqrt(1 + 0.25 * std::pow(std::tan(delta),2));
  }
  float inline ToBase(const float &v, const float &u0)
  {
    return v / std::sqrt(1 + 0.25 * std::pow(std::tan(u0),2));
  }
private:
  Spline2D track_;
  Vec_f last_result_;
  bool first_slove_;//小车从零速后第一次启动求解
};
//
class FG_EVAL{
public:
  // Eigen::VectorXd coeeffs;
  M_XREF traj_ref;
  //状态量偏差的系数
  Eigen::Matrix<float,4,SIZE_PREDICTION> Q;
  //控制量的系数
  Eigen::Matrix<float,2,SIZE_PREDICTION-1> R;
  //控制量变化率的系数
  Eigen::Matrix<float,2,SIZE_PREDICTION-2> Rd;
  FG_EVAL(M_XREF traj_ref)
  {
    this->traj_ref = traj_ref;
    Q = Eigen::Matrix<float,4,SIZE_PREDICTION>::Zero();
    R = Eigen::Matrix<float,2,SIZE_PREDICTION-1>::Zero();
    Rd = Eigen::Matrix<float,2,SIZE_PREDICTION-2>::Zero();
  }
  using ADvector = CPPAD_TESTVECTOR(AD<double>);

  void operator()(ADvector& fg, const ADvector& vars){
    //初始化代价函数的系数矩阵Q,R,Rd
    for(int i=0;i<SIZE_PREDICTION;i++)
    {
      Q(0,i)= 1.0;
      Q(1,i)= 3.0;
      Q(2,i)= 4.0;
      Q(3,i)= 2.0;
    }
    for(int j=0;j<SIZE_PREDICTION-1;j++)
    {
      R(0,j) = 1.0;
      R(1,j) = 10.0;
    }
    for(int k=0;k<SIZE_PREDICTION-2;k++)
    {
      Rd(0,k) = 0.1;
      Rd(1,k) = 5.0;
    }
    fg[0] = 0;
  //fg[0]为代价函数
  //fg[1],fg[2]...依次为约束表达式，如状态量满足xmin<x<xmax,共有size个时刻,控制量满足umin<u<umax，共有size-1个时刻
  //关于加速度和车头转角的代价函数
    for(int i=0; i<SIZE_PREDICTION-1; i++)
    {
      fg[0] +=  R(0,i) * CppAD::pow(vars[default_param_.a_start+i], 2);
      fg[0] +=  R(1,i) * CppAD::pow(vars[default_param_.delta_start+i], 2);
      //std::cout<<"×××××××××××××××第"<<i<<"时刻×××××××××××××××××"<<std::endl;
      //std::cout<<"ua ="<<CppAD::pow(vars[default_param_.a_start+i], 2)<<std::endl;
      //std::cout<<"udelta ="<<vars[default_param_.delta_start+i]<<std::endl;
    }
//    std::cout<<"cost_r"<<fg[0]<<std::endl;
    //关于加速度和车头转角变化率的代价函数
    for(int i=0; i<SIZE_PREDICTION-2; i++)
    {
      fg[0] += Rd(0,i) * CppAD::pow(vars[default_param_.a_start+i+1] - vars[default_param_.a_start+i], 2);
      fg[0] += Rd(1,i) * CppAD::pow(vars[default_param_.delta_start+i+1] - vars[default_param_.delta_start+i], 2);
//      std::cout<<"×××××××××××××××第"<<i<<"时刻×××××××××××××××××"<<std::endl;
//      std::cout<<"dua ="<<CppAD::pow(vars[default_param_.a_start+i+1] - vars[default_param_.a_start+i], 2)<<std::endl;
//      std::cout<<"dudelta ="<<CppAD::pow(vars[default_param_.delta_start+i+1] - vars[default_param_.delta_start+i], 2)<<std::endl;
    }
//    std::cout<<"cost_rd+cost_r"<<fg[0]<<std::endl;
    //控制量变化率的约束a<u(t+1)-u(t)<b
    for(int i=0; i<SIZE_PREDICTION-2; i++)
    {
      fg[1+default_param_.a_start+i] = vars[default_param_.a_start+i+1] - vars[default_param_.a_start+i];
      fg[default_param_.delta_start+i] = vars[default_param_.delta_start+i+1] - vars[default_param_.delta_start+i];
    }

    // 0时刻状态量可以作为一个等式约束
    fg[1 + default_param_.x_start]   = vars[default_param_.x_start];
    fg[1 + default_param_.y_start]   = vars[default_param_.y_start];
    fg[1 + default_param_.yaw_start] = vars[default_param_.yaw_start];
    fg[1 + default_param_.v_start]   = vars[default_param_.v_start];

    // The rest of the constraints
    for (int i = 0; i <  SIZE_PREDICTION- 1; i++)
    {
      // The state at time t+1 .
      AD<double> x1   = vars[default_param_.x_start + i + 1];
      AD<double> y1   = vars[default_param_.y_start + i + 1];
      AD<double> yaw1 = vars[default_param_.yaw_start + i + 1];
      AD<double> v1   = vars[default_param_.v_start + i + 1];

      // The state at time t.
      AD<double> x0   = vars[default_param_.x_start + i];
      AD<double> y0   = vars[default_param_.y_start + i];
      AD<double> yaw0 = vars[default_param_.yaw_start + i];
      AD<double> v0   = vars[default_param_.v_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[default_param_.delta_start + i];
      AD<double> a0 = vars[default_param_.a_start + i];

      // constraint with the dynamic model
      fg[2 + default_param_.x_start + i]   = x1 - (x0 + v0 * CppAD::cos(yaw0) * DT);
      fg[2 + default_param_.y_start + i]   = y1 - (y0 + v0 * CppAD::sin(yaw0) * DT);
      fg[2 + default_param_.yaw_start + i] = yaw1 - (yaw0 + v0 * CppAD::tan(delta0) / default_param_.wheel_distance * DT);
      fg[2 + default_param_.v_start + i]   = v1 - (v0 + a0 * DT);

      AD<double> ex     = traj_ref(0, i+1) - vars[default_param_.x_start + i+1];
      AD<double> ey     = traj_ref(1, i+1) - vars[default_param_.y_start + i+1];
      AD<double> etheta = traj_ref(2, i+1) - vars[default_param_.yaw_start + i+1];
      //增加预定轨迹相对于当前方向的角度差
      //AD<double> etheta1 = traj_ref(2, i+1) - vars[default_param_.yaw_start];
      AD<double> ec     = CppAD::sqrt(CppAD::pow(ex,2)+CppAD::pow(ey,2));//横向偏差
      AD<double> el     = 0.0;
      if(etheta > M_PI)
        etheta -= 2*M_PI;
      if(etheta < -M_PI)
        etheta += 2*M_PI;
//      if(etheta1 > M_PI)
//        etheta1 -= 2*M_PI;
//      if(etheta1 < -M_PI)
//        etheta1 += 2*M_PI;

      fg[0] += Q(0,i) * CppAD::pow(el, 2);
      fg[0] += Q(1,i) * CppAD::pow(ec, 2);
      fg[0] += Q(2,i) * CppAD::pow(etheta, 2);
      //fg[0] += 0.1 * CppAD::pow(etheta1, 2);
      fg[0] += Q(3,i) * CppAD::pow(traj_ref(3, i) - v0, 2);
//      std::cout<<"××第"<<i<<"时刻××"<<std::endl;
//      std::cout<<"el ="<<el<<std::endl;
//      std::cout<<"ec ="<<ec<<std::endl;
//      std::cout<<"ex ="<<ex<<std::endl;
//      std::cout<<"ey ="<<ey<<std::endl;
//      std::cout<<"etheta ="<<etheta<<std::endl;
//      std::cout<<"etheta1 ="<<etheta1<<std::endl;
//      std::cout<<"dv ="<<CppAD::pow(traj_ref(3, i) - v0, 2)<<std::endl;
    }
//    std::cout<<"cost_rd+cost_r+cost_q"<<fg[0]<<std::endl;
  }
};
}
