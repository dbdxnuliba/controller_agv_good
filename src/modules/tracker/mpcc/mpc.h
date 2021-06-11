// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#pragma once

#include "config.h"
#include "types.h"
#include "params.h"
#include "arc_length_spline.h"
#include "model.h"
#include "integrator.h"
#include "cost.h"
#include "constraints.h"
#include "bounds.h"

#include "solver_interface.h"
#include "hpipm_interface.h"

#include <array>
#include <memory>
#include <ctime>
#include <ratio>
#include <chrono>
#include <boost/algorithm/clamp.hpp>
namespace mpcc{
/**
 * @brief 预测出k时刻的状态量和控制量
 */
struct OptVariables
{
  State xk;
  Input uk;
};
/**
 * @brief 定义进行优化计算输入量的结构体
 */
struct Stage
{
  LinModelMatrix lin_model;
  CostMatrix cost_mat;
  ConstrainsMatrix constrains_mat;

  Bounds_x u_bounds_x;
  Bounds_x l_bounds_x;

  Bounds_u u_bounds_u;
  Bounds_u l_bounds_u;

  Bounds_s u_bounds_s;
  Bounds_s l_bounds_s;

  //nx表示状态量个数
  //nu表示控制量个数
  //nbx表示状态量的边界个数
  //nbu表示控制量的边界个数
  //ng表示多体约束的数量
  //ns表示软约束的个数
  int nx, nu, nbx, nbu, ng, ns;
};
/**
 * @brief 定义存放MPC控制器的计算结果的结构体
 */
struct MPCReturn
{
  Input u0;
  //长度为N+1，元素类型为OptVariables的数组
  std::array<OptVariables,N+1> mpc_horizon;
  double time_total;
};

class MPC
{
public:
  MPC();
  MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts,const PathToJson &path);
  /**
   * @brief  执行mpc算法的主循环,最终返回需要的结果
   * @param  输入参数:当前状态量
   * @param  输出参数:无
   * @return 返回值为指定格式的数据,包括N+1个时刻的预测值,以及当前时刻的控制量
   */
  MPCReturn RunMPC(State &x0);
  /**
     * @brief 对路径进行三次样条插值
     * @param 输入参数:路径点xy坐标
     * @param 输出参数:无
     * @return 返回值:无
     */
  void SetTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y);
  /**
     * @brief  打印输入状态量
     * @param  输入参数:状态量向量
     * @param  输出参数:无
     * @return 返回值:无
     */
  void PrintState(State x);
private:
  /**
   * @brief  将求解需要的N个时刻初始条件进行打包
   * @param  输入参数:无
   * @param  输出参数:无
   * @return 返回值:无
   */
  void SetMPCProblem();
  /**
   * @brief  将求解需要的每一时刻初始值进行打包
   * @param  输入参数:某一时刻初始状态量xk,控制量uk,对应的预测时间编号(0,1,...,N)
   * @param  输出参数:无
   * @return 返回值:无
   */
  void SetStage(const State &xk, const Input &uk, int time_step);
  /**
   * @brief  对代价函数矩阵进行正规化处理
   * @param  输入参数:
   * @param  输出参数:
   * @return 返回值:
   */
  CostMatrix NormalizeCost(const CostMatrix &cost_mat);
  /**
   * @brief  对汽车模型状态空间模型系数矩阵进行正规化处理
   * @param  输入参数:
   * @param  输出参数:
   * @return 返回值:
   */
  LinModelMatrix NormalizeDynamics(const LinModelMatrix &lin_model);
  /**
   * @brief  对约束系数矩阵进行正规化处理
   * @param  输入参数:
   * @param  输出参数:
   * @return 返回值:
   */
  ConstrainsMatrix NormalizeCon(const ConstrainsMatrix &con_mat);
  /**
   * @brief  对求解结果进行反正规化处理
   * @param  输入参数:求解出的N+1个时刻的结果
   * @param  输出参数:无
   * @return 返回值:经过正规化处理后的结果
   */
  std::array<OptVariables,N+1> DeNormalizeSolution(const std::array<OptVariables,N+1> &solution);
  /**
   * @brief  后期进行求解运算时需要对求解输入进行更新
   * @param  输入参数:当前时刻状态量
   * @param  输出参数:无
   * @return 返回值:无
   */
  void UpdateInitialGuess(const State &x0);
  /**
   * @brief  初次进行求解运算时对需要的求解输入进行初始化
   * @param  输入参数:当前时刻状态量
   * @param  输出参数:无
   * @return 返回值:无
   */
  void GenerateNewInitialGuess(const State &x0);
  /**
   * @brief  当初始化的结果超出正常范围,就需要进行正规化处理
   * @param  输入参数:无
   * @param  输出参数:无
   * @return 返回值:无
   */
  void UnwrapInitialGuess();
  /**
   * @brief
   * @param
   * @param
   * @return
   */
  std::array<OptVariables, N + 1> SqpSolutionUpdate(const std::array<OptVariables, N + 1> &last_solution,
                                                    const std::array<OptVariables, N + 1> &current_solution);
  /**
   * @brief
   * @param
   * @param
   * @return
   */
  bool IsGoReached();
  /**
   * @brief  为防止大角度偏转不能及时回调,需要在这种情况下对速度进行消减
   * @param  输入参数:initial_guess_
   * @param  输出参数:无
   * @return 返回值:消减系数
   */
  double ComputeKCurvature(const OptVariables &initial);
  /**
   * @brief  解决到终点前的停车问题
   * @param  输入参数:initial_guess_
   * @param  输出参数:无
   * @return 返回值:消减系数
   */
  double ComputekFinal(const OptVariables &initial);
  /**
   * @brief  将所有时刻的变量置为0
   * @param  输入参数:无
   * @param  输出参数:无
   * @return 返回值:MPCReturn
   */
  MPCReturn SetZero(State x);
  /**
   * @brief  将所有时刻的变量置为0
   * @param  输入参数:无
   * @param  输出参数:无
   * @return 返回值:MPCReturn
   */
  void SmoothSteer(std::array<OptVariables, N + 1> &current_solution);
private:
  bool valid_initial_guess_;

  std::array<Stage, N + 1> stages_;

  std::array<OptVariables, N + 1> initial_guess_;
  std::array<OptVariables, N + 1> optimal_solution_;

  int n_sqp_;//在同一时刻连续计算的次数，保证至少一次有优化结果
  double sqp_mixing_;//预测值与计算结果的比重，目前是计算结果占80%
  int n_non_solves_;//当n_sqp_次都没有计算结果，给n_non_solves_加1
  int n_no_solves_sqp_;//当每次没有计算结果时，n_no_solves_sqp_加1，达到阈值，n_sqp_加1
  int n_reset_;//n_non_solves_次数达到5次后，认为连续5个时刻都没有计算结果

  const double Ts_;//周期为0.04s

  Model model_;//汽车模型的类
  Integrator integrator_;//定义一些数学方法，比如线性化方法，离散化算法
  Cost cost_;//路径规划类，暂时用不到
  Constraints constraints_;//规定代价函数的约束
  ArcLengthSpline track_;//处理路径的类

  Bounds bounds_;//定义状态量和控制量的上下限
  NormalizationParam normalization_param_;//
  Param param_;

  std::unique_ptr<SolverInterface> solver_interface_;
};

}
