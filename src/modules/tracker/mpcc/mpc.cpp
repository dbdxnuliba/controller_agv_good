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

#include "mpc.h"

namespace mpcc{
MPC::MPC()
:Ts_(1.0)
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

MPC::MPC(int n_sqp, int n_reset,double sqp_mixing, double Ts,const PathToJson &path)
:Ts_(Ts),
valid_initial_guess_(false),
solver_interface_(new HpipmInterface()),
param_(Param(path.param_path)),
normalization_param_(NormalizationParam(path.normalization_path)),
bounds_(BoundsParam(path.bounds_path)),
constraints_(Constraints(Ts,path)),
cost_(Cost(path)),
integrator_(Integrator(Ts,path)),
model_(Model(Ts,path)),
track_(ArcLengthSpline(path))
{
    n_sqp_ = n_sqp;//2
    sqp_mixing_ = sqp_mixing;
    n_non_solves_ = 0;
    n_no_solves_sqp_ = 0;
    n_reset_ = n_reset;//5
}
//对需要优化的问题进行设置
void MPC::SetMPCProblem()
{
    for(int i=0;i<=N;i++)
    {
        SetStage(initial_guess_[i].xk,initial_guess_[i].uk,i);
    }
}
//设置优化过程中的各种约束上下限，模型
void MPC::SetStage(const State &xk, const Input &uk, const int time_step)
{
  stages_[time_step].nx = NX;
  stages_[time_step].nu = NU;

  if(time_step == 0)
  {
    stages_[time_step].ng = 0;
    stages_[time_step].ns = 0;
  }
  else
  {
    stages_[time_step].ng = NPC;
    stages_[time_step].ns = NS;
  }

  State xk_nz = xk;
  xk_nz.vxNonZero(param_.vx_zero);//v_x小于0.3，置为0.3
  stages_[time_step].cost_mat = NormalizeCost(cost_.getCost(track_,xk_nz,time_step));
  stages_[time_step].lin_model = NormalizeDynamics(model_.getLinModel(xk_nz,uk));
  stages_[time_step].constrains_mat = NormalizeCon(constraints_.getConstraints(track_,xk_nz,uk));
  //todo设置状态，控制，以及轨迹的约束，为什么要加系数矩阵，是为了权重系数？？？？？？？？？？？？？？
  //包括代价函数矩阵,还有约束矩阵乘系数矩阵,猜测是为了减小搜索范围,加速计算
  stages_[time_step].l_bounds_x = normalization_param_.T_x_inv*bounds_.getBoundsLX();
  stages_[time_step].u_bounds_x = normalization_param_.T_x_inv*bounds_.getBoundsUX();
  stages_[time_step].l_bounds_u = normalization_param_.T_u_inv*bounds_.getBoundsLU();
  stages_[time_step].u_bounds_u = normalization_param_.T_u_inv*bounds_.getBoundsUU();
  stages_[time_step].l_bounds_s = normalization_param_.T_s_inv*bounds_.getBoundsLS();
  stages_[time_step].u_bounds_s = normalization_param_.T_s_inv*bounds_.getBoundsUS();
  //需要将状态量中的s进行单独处理
  stages_[time_step].l_bounds_x(si_index.s) = normalization_param_.T_x_inv(si_index.s,si_index.s)*
      (initial_guess_[time_step].xk.s - param_.s_trust_region);
  stages_[time_step].u_bounds_x(si_index.s) = normalization_param_.T_x_inv(si_index.s,si_index.s)*
      (initial_guess_[time_step].xk.s + param_.s_trust_region);
  //todo要不要根据s反向缩小x,y的上下边界
  Eigen::Vector2d ref_pos = track_.getPostion(initial_guess_[time_step].xk.s);
  stages_[time_step].l_bounds_x(si_index.X) = normalization_param_.T_x_inv(si_index.X,si_index.X)*
      (ref_pos(0) - 1.0);
  stages_[time_step].u_bounds_x(si_index.X) = normalization_param_.T_x_inv(si_index.X,si_index.X)*
      (ref_pos(0) + 1.0);

  stages_[time_step].l_bounds_x(si_index.Y) = normalization_param_.T_x_inv(si_index.Y,si_index.Y)*
      (ref_pos(1) - 1.0);
  stages_[time_step].u_bounds_x(si_index.Y) = normalization_param_.T_x_inv(si_index.Y,si_index.Y)*
      (ref_pos(1) + 1.0);
}

CostMatrix MPC::NormalizeCost(const CostMatrix &cost_mat)
{
  const Q_MPC Q = normalization_param_.T_x*cost_mat.Q*normalization_param_.T_x;
  const R_MPC R = normalization_param_.T_u*cost_mat.R*normalization_param_.T_u;
  const q_MPC q = normalization_param_.T_x*cost_mat.q;
  const r_MPC r = normalization_param_.T_u*cost_mat.r;
  const Z_MPC Z = normalization_param_.T_s*cost_mat.Z*normalization_param_.T_s;
  const z_MPC z = normalization_param_.T_s*cost_mat.z;
  return {Q,R,S_MPC::Zero(),q,r,Z,z};
}

LinModelMatrix MPC::NormalizeDynamics(const LinModelMatrix &lin_model)
{
    const A_MPC A = normalization_param_.T_x_inv*lin_model.A*normalization_param_.T_x;
    const B_MPC B = normalization_param_.T_x_inv*lin_model.B*normalization_param_.T_u;
    const g_MPC g = normalization_param_.T_x_inv*lin_model.g;
    return {A,B,g};
}

ConstrainsMatrix MPC::NormalizeCon(const ConstrainsMatrix &con_mat)
{
    const C_MPC C = con_mat.C*normalization_param_.T_x;
    const D_MPC D =  con_mat.D*normalization_param_.T_u;
    const d_MPC dl = con_mat.dl;
    const d_MPC du = con_mat.du;
    return {C,D,dl,du};
}

std::array<OptVariables,N+1> MPC::DeNormalizeSolution(const std::array<OptVariables,N+1> &solution)
{
    std::array<OptVariables, N + 1> denormalized_solution;
    StateVector updated_x_vec;
    InputVector updated_u_vec;
    for (int i = 0; i <= N; i++) {
        updated_x_vec = normalization_param_.T_x*stateToVector(solution[i].xk);
        updated_u_vec = normalization_param_.T_u*inputToVector(solution[i].uk);

        denormalized_solution[i].xk = vectorToState(updated_x_vec);
        denormalized_solution[i].uk = vectorToInput(updated_u_vec);
    }
    return denormalized_solution;
}

//对状态以及控制量的预测值进行更新
void MPC::UpdateInitialGuess(const State &x0)
{
  //如果因为障碍物等原因出现中途停车,需要特殊处理
  if(x0.vs == 0.0 && x0.delta != 0.0 && x0.r != 0.0)
  {
    initial_guess_[0].xk = x0;
    initial_guess_[0].uk.setZero();
    initial_guess_[0].xk.D = 0.5;
    initial_guess_[0].xk.vs = param_.initial_velocity;
    initial_guess_[0].xk.vx = param_.initial_velocity;
    for(int i = 1;i<=N;i++)
    {
      initial_guess_[i].xk = initial_guess_[0].xk;
      initial_guess_[i].uk = initial_guess_[0].uk;

      initial_guess_[i].xk.s = initial_guess_[i-1].xk.s + Ts_* x0.vs;
      Eigen::Vector2d track_pos_i = track_.getPostion(initial_guess_[i].xk.s);
      initial_guess_[i].xk.X = track_pos_i(0);
      initial_guess_[i].xk.Y = track_pos_i(1);
      initial_guess_[i].xk.phi = initial_guess_[i-1].xk.phi;
      initial_guess_[i].xk.vx = x0.vs;
      initial_guess_[i].xk.vy = 0.5 * x0.vs * tan(x0.delta);
      initial_guess_[i].xk.vs = x0.vs;
      initial_guess_[i].xk.delta = x0.delta;
    }
  }
  else
  {
    //上一时刻的对i的预测作为对这一时刻i-1的预测
    for(int i=1;i<N;i++)
      initial_guess_[i-1] = initial_guess_[i];
    //更新当前时刻的预测
    initial_guess_[0].xk = x0;
    initial_guess_[0].uk.setZero();
    //todo 为什么不用N时刻对N-1时刻赋值，是否是有什特例还没考虑到
    //initial_guess_[N-1].xk = initial_guess_[N-2].xk;
    //initial_guess_[N-1].uk = initial_guess_[N-2].uk;
    //利用4阶龙哥库塔法更新下一时刻状态
    initial_guess_[N].xk = integrator_.RK4(initial_guess_[N-1].xk,initial_guess_[N-1].uk,Ts_);
    initial_guess_[N].uk.setZero();
  }
  UnwrapInitialGuess();
}

// alternatively OptVariables MPC::unwrapInitialGuess(const OptVariables &initial_guess)
void MPC::UnwrapInitialGuess()
{
  double L = track_.getLength();
  for(int i=1;i<=N;i++)
  {
    if((initial_guess_[i].xk.phi - initial_guess_[i-1].xk.phi) < -M_PI)
    {
      initial_guess_[i].xk.phi += 2.*M_PI;
    }
    if((initial_guess_[i].xk.phi - initial_guess_[i-1].xk.phi) > M_PI)
    {
      initial_guess_[i].xk.phi -= 2.*M_PI;
    }
    if(initial_guess_[i].xk.vs < 0.1)
    {
      initial_guess_[i].xk.vs = 0.1;
      initial_guess_[i].xk.vx = 0.1;
    }
    //保证在终点不会计算出s>L
    if(initial_guess_[i].xk.s > L)
    {
      //initial_guess_[i].xk.s -= L;
      initial_guess_[i].xk.s = L;
    }
    //保证切路时,xk.s不会与x0.s差距太大,暂定0.5m
    if((initial_guess_[i].xk.s - initial_guess_[i-1].xk.s) > 0.5)
    {
      //initial_guess_[i].xk.s -= L;
      initial_guess_[i].xk.s = initial_guess_[i-1].xk.s + initial_guess_[i-1].xk.vs * Ts_;
    }
    //当s=L,速度和角速度置为0,变化率也置为0
    if(initial_guess_[i].xk.s == L)
    {
      initial_guess_[i].xk.r  = 0;
      initial_guess_[i].xk.vx = 0;
      initial_guess_[i].xk.vy = 0;
      initial_guess_[i].xk.D  = 0;
      initial_guess_[i].xk.vs = 0;

      initial_guess_[i].uk.dD     = 0;
      initial_guess_[i].uk.dDelta = 0;
      initial_guess_[i].uk.dVs    = 0;

      //std::cout<<"即将到达终点"<<std::endl;
    }
  }
}
//如果没有初始化过状态，就需要进行第一次更新
void MPC::GenerateNewInitialGuess(const State &x0)
{
    //当前时刻t=0,状态和控制量如下
    initial_guess_[0].xk = x0;
    initial_guess_[0].uk.setZero();
    //按照初始状态量更新N个时刻的量
//    if(x0.D == 0.5 && x0.r == 0.0)
//    {
//      for(int i = 1;i<=N;i++)
//      {
//        initial_guess_[i].xk.setZero();
//        initial_guess_[i].uk.setZero();

//        initial_guess_[i].xk.s = initial_guess_[i-1].xk.s + Ts_*param_.initial_velocity;
//        Eigen::Vector2d track_pos_i = track_.getPostion(initial_guess_[i].xk.s);
//        Eigen::Vector2d track_dpos_i = track_.getDerivative(initial_guess_[i].xk.s);
//        initial_guess_[i].xk.X = track_pos_i(0);
//        initial_guess_[i].xk.Y = track_pos_i(1);
//        initial_guess_[i].xk.phi = atan2(track_dpos_i(1),track_dpos_i(0));
//        initial_guess_[i].xk.vx = param_.initial_velocity;
//        initial_guess_[i].xk.vs = param_.initial_velocity;
//      }
//      std::cout<<"第一次进行求解,开始初始化"<<std::endl;
//    }
//    else
//    {
//      for(int i = 1;i<=N;i++)
//      {
//        initial_guess_[i].xk.setZero();
//        initial_guess_[i].uk.setZero();

//        // reference path derivatives
//        Eigen::Vector2d dpos_ref = track_.getDerivative(initial_guess_[i-1].xk.s);
//        const double theta_ref = atan2(dpos_ref(1),dpos_ref(0));
//        initial_guess_[i].xk.s = initial_guess_[i-1].xk.s + Ts_* x0.vs * cos(initial_guess_[i-1].xk.phi - theta_ref);
//        Eigen::Vector2d track_pos_i = track_.getPostion(initial_guess_[i].xk.s);
//        initial_guess_[i].xk.X = track_pos_i(0);
//        initial_guess_[i].xk.Y = track_pos_i(1);
//        initial_guess_[i].xk.phi = initial_guess_[i-1].xk.phi + Ts_* initial_guess_[i-1].xk.r;
//        initial_guess_[i].xk.vx = x0.vs;
//        initial_guess_[i].xk.vy = 0.5 * x0.vs * tan(x0.delta);
//        initial_guess_[i].xk.vs = x0.vs;
//        initial_guess_[i].xk.delta = x0.delta;
//        initial_guess_[i].xk.r = x0.r;
//      }
//      std::cout<<"多次求解失败,重新初始化"<<std::endl;
//    }



    //按照初始状态量更新N个时刻的量
    if(x0.D == 0.5 && x0.r == 0.0)
    {
      for(int i = 1;i<=N;i++)
      {
        initial_guess_[i].xk.setZero();
        initial_guess_[i].uk.setZero();

        initial_guess_[i].xk.s = initial_guess_[i-1].xk.s + Ts_*param_.initial_velocity;
        Eigen::Vector2d track_pos_i = track_.getPostion(initial_guess_[i].xk.s);
        Eigen::Vector2d track_dpos_i = track_.getDerivative(initial_guess_[i].xk.s);
        initial_guess_[i].xk.X = track_pos_i(0);
        initial_guess_[i].xk.Y = track_pos_i(1);
        initial_guess_[i].xk.phi = atan2(track_dpos_i(1),track_dpos_i(0));
        initial_guess_[i].xk.vx = param_.initial_velocity;
        initial_guess_[i].xk.vs = param_.initial_velocity;
      }
      std::cout<<"第一次进行求解,开始初始化"<<std::endl;
    }
    else
    {
      for(int i = 1;i<=N;i++)
      {
        initial_guess_[i].xk.setZero();
        initial_guess_[i].uk.setZero();

        // reference path derivatives
        Eigen::Vector2d dpos_ref = track_.getDerivative(initial_guess_[i-1].xk.s);
        const double theta_ref = atan2(dpos_ref(1),dpos_ref(0));
        initial_guess_[i].xk.s = initial_guess_[i-1].xk.s + Ts_* param_.initial_velocity * cos(initial_guess_[i-1].xk.phi - theta_ref);
        Eigen::Vector2d track_pos_i = track_.getPostion(initial_guess_[i].xk.s);
        initial_guess_[i].xk.X = track_pos_i(0);
        initial_guess_[i].xk.Y = track_pos_i(1);
        initial_guess_[i].xk.phi = initial_guess_[i-1].xk.phi + Ts_* initial_guess_[i-1].xk.r;
        initial_guess_[i].xk.vx = param_.initial_velocity;
        initial_guess_[i].xk.vy = 0.5 * param_.initial_velocity * tan(x0.delta);
        initial_guess_[i].xk.vs = param_.initial_velocity;
        initial_guess_[i].xk.delta =  initial_guess_[i-1].xk.delta;
        initial_guess_[i].xk.r = initial_guess_[i-1].xk.r;
      }
      std::cout<<"多次求解失败,重新初始化"<<std::endl;
    }
    UnwrapInitialGuess();
    valid_initial_guess_ = true;
}
//根据上一时刻和当前时刻的求解结果，进行结果更新，按照两个时刻的结果进行比例混合
std::array<OptVariables,N+1> MPC::SqpSolutionUpdate(const std::array<OptVariables,N+1> &last_solution,
                                                    const std::array<OptVariables,N+1> &current_solution)
{
    //TODO use line search and merit function
    std::array<OptVariables,N+1> updated_solution;
    StateVector updated_x_vec;
    InputVector updated_u_vec;

    for(int i = 0;i<=N;i++)
    {
        updated_x_vec = sqp_mixing_*stateToVector(current_solution[i].xk)
                        +(1.0-sqp_mixing_)*stateToVector(last_solution[i].xk);
        updated_u_vec = sqp_mixing_*inputToVector(current_solution[i].uk)
                        +(1.0-sqp_mixing_)*inputToVector(last_solution[i].uk);
        updated_solution[i].xk = vectorToState(updated_x_vec);
        updated_solution[i].uk = vectorToInput(updated_u_vec);
    }
    //考虑在过圆弧或是将要到达终点时,降低vs以防止偏离路线太远
    double kdelta = ComputeKCurvature(initial_guess_[0]);//根据当前路线的曲率给出消减系数
    double kfinal = ComputekFinal(initial_guess_[0]);//根据到终点的距离给出消减系数
    for(int i = 0;i<=N;i++)
    {
      double v = kdelta * kfinal * updated_solution[i].xk.vs;
      updated_solution[i].xk.vs = clamp(v,0.1,1.0);
    }
    //smooth steer angle
    updated_solution[1].xk.delta = clamp(updated_solution[1].xk.delta,current_solution[0].xk.delta-0.3*Ts_,
        current_solution[0].xk.delta+0.3*Ts_);
    updated_solution[1].xk.vs = clamp(updated_solution[1].xk.vs,current_solution[0].xk.vs-0.5*Ts_,
        current_solution[0].xk.vs+0.5*Ts_);
    return updated_solution;
}

double MPC::ComputeKCurvature(const OptVariables &initial)
{
  //计算弧长对应的曲率
  double sk = track_.ComputeCurvature(initial.xk.s);
  double k = 1;
  if(std::fabs(sk)>0.2)
    k = 1/std::exp(std::fabs(sk));
  clamp(k,0.5,1.0);
  std::cout<<"sk:"<<sk<<"k:"<<k<<std::endl;
  return k;
}

double MPC::ComputekFinal(const OptVariables &initial)
{
  double s1 = track_.porjectOnSpline(initial.xk);
  double s2 = track_.getLength();
  double k  = (s2 - s1) / 2.5;
  clamp(k,0.3,1.0);
//  if(k < 1)
//  {
//    std::cout<<"ComputekFinal:"<<k<<"  s1:"<<s1<<"  s2:"<<s2<<std::endl;
//  }
  return k;
}

MPCReturn MPC::SetZero(State x)
{
  MPCReturn m1;
  for(int i=0;i<m1.mpc_horizon.size();i++)
  {
    m1.mpc_horizon.at(i).xk.X      = x.X;
    m1.mpc_horizon.at(i).xk.Y      = x.Y;
    m1.mpc_horizon.at(i).xk.phi    = x.phi;
    m1.mpc_horizon.at(i).xk.vx     = 0.0;
    m1.mpc_horizon.at(i).xk.vy     = 0.0;
    m1.mpc_horizon.at(i).xk.r      = 0.0;
    m1.mpc_horizon.at(i).xk.s      = 0.0;
    m1.mpc_horizon.at(i).xk.vs     = 0.0;
    m1.mpc_horizon.at(i).xk.delta  = 0.0;
    m1.mpc_horizon.at(i).xk.D      = 0.5;
    m1.mpc_horizon.at(i).uk.dD     = 0.0;
    m1.mpc_horizon.at(i).uk.dVs    = 0.0;
    m1.mpc_horizon.at(i).uk.dDelta = 0.0;
  }
  m1.time_total = 0.0;
  m1.u0.dD      = 0.0;
  m1.u0.dDelta  = 0.0;
  m1.u0.dVs     = 0.0;
  std::cout<<"到达终点,不需要执行mpc"<<std::endl;
  return m1;
}

void MPC::SmoothSteer(std::array<OptVariables, N+1> &current_solution)
{

}

MPCReturn MPC::RunMPC(State &x0)
{
    if(track_.IsGoReached(x0))
    {
      MPCReturn m1 = SetZero(x0);
      return m1;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    //定义求解状态，0为正常状态
    int solver_status = -1;
    //根据机器人状态计算新的弧长，并对切向角和弧长进行处理
    //保证当路径更新后,重新将x0.s置为0
    x0.s = track_.porjectOnSpline(x0);
    x0.unwrap(track_.getLength());
    //如果已经进行了初始化，只需要更新
    if(valid_initial_guess_)
        UpdateInitialGuess(x0);
    //需要进行初始化
    else
        GenerateNewInitialGuess(x0);
    //PrintState(x0);
    //todo通过计数处理求解错误的情况，超过阈值次数进行处理
    //n_reset_最大为5，连续5个时刻没有求解出结果，就需要重新初始化
    //n_sqp_最大为2，同一时刻连续计算两次，需要保证至少有一次能求解出结果
    n_no_solves_sqp_ = 0;
    for(int i=0;i<n_sqp_;i++)
    {
        //对需要优化的问题进行设置，约束，模型等等
        SetMPCProblem();
        //状态量被一个权重矩阵做了正规化处理
        State x0_normalized = vectorToState(normalization_param_.T_x_inv*stateToVector(x0));
        //进行MPC求解
        optimal_solution_ = solver_interface_->solveMPC(stages_,x0_normalized, &solver_status);
        //求出结果，进行了反正规化处理
        optimal_solution_ = DeNormalizeSolution(optimal_solution_);
        //0为正常，1为达到最大迭代次数，2是什么？？？
        //if(solver_status != 0)
        if(solver_status > 1)
        {
           n_no_solves_sqp_++;
           std::cout<<"solver_status="<<solver_status<<std::endl;
        }
        if(solver_status <= 1)
            initial_guess_ = SqpSolutionUpdate(initial_guess_,optimal_solution_);
    }
    const int max_error = std::max(n_sqp_-1,1);//1
    //如果所有计算次数都没有求出结果，n_non_solves_次数加1
    if(n_no_solves_sqp_ >= max_error)
    {
      n_non_solves_++;
    }
    else
    {
      n_non_solves_ = 0;
      //std::cout<<"求解成功"<<std::endl;
    }
    //连续3个循环没有求出结果，需要初始化状态量，而不是根据当前时刻进行更新
    if(n_non_solves_ >= n_reset_){
        valid_initial_guess_ = false;
    }
    //计算总的优化时间
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    double time_nmpc = time_span.count();//总的优化时间

    return {initial_guess_[0].uk,initial_guess_,time_nmpc};
}
//生成二维样条曲线
void MPC::SetTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y)
{
  //std::cout<<"路径规划输入的路径长度为:"<<X.size()<<std::endl;
  track_.gen2DSpline(X,Y);
}

void MPC::PrintState(State x)
{
  std::cout<<"机器人位置X是:"<<x.X<<std::endl;
  std::cout<<"机器人位置Y是:"<<x.Y<<std::endl;
  std::cout<<"机器人位置PHI是:"<<x.phi<<std::endl;
  //std::cout<<"机器人线速度vx是:"<<x.vx<<std::endl;
  //std::cout<<"机器人线速度vy是:"<<x.vy<<std::endl;
  std::cout<<"机器人线速度r是:"<<x.r<<std::endl;
  std::cout<<"机器人走过的长度s是:"<<x.s<<std::endl;
  //std::cout<<"控制指令D是:"<<x.D<<std::endl;
  std::cout<<"前轮转角delta是:"<<x.delta<<std::endl;
  std::cout<<"后轮速度vs是:"<<x.vs<<std::endl;
  std::cout<<"************************"<<std::endl;
}
}
