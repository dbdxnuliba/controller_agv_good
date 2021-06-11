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
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <blasfeo_d_aux_ext_dep.h>
#include "hpipm_d_ocp_qp_ipm.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "hpipm_timing.h"

#include "config.h"
#include "types.h"
#include "model.h"
#include "cost.h"
#include "constraints.h"
#include "bounds.h"
#include "solver_interface.h"

#include <array>
#include <vector>

namespace mpcc {
struct OptVariables;
struct Stage;
//HpipmBound 和 stages bounds 不同，需要储存在指针可以指向的地方
struct HpipmBound {
    std::vector<int> idx_u;//某一时刻控制量有效边界条件的编号集合
    std::vector<int> idx_x;//某一时刻状态量有效边界条件的编号集合
    std::vector<int> idx_s;//某一时刻软约束的编号集合
    std::vector<double> lower_bounds_u;//某一时刻控制量下边界条件
    std::vector<double> upper_bounds_u;//某一时刻控制量上边界条件
    std::vector<double> lower_bounds_x;//某一时刻状态量下边界条件
    std::vector<double> upper_bounds_x;//某一时刻状态量上边界条件
};

class HpipmInterface : public SolverInterface {
public:
    std::array<OptVariables,N+1> solveMPC(std::array<Stage,N+1> &stages,const State &x0, int *status);

    ~HpipmInterface(){
        std::cout << "Deleting Hpipm Interface" << std::endl;
    }
private:
    int nx_[N+1];//保存每个时刻状态量的个数
    int nu_[N+1];//保存每个时刻控制量的个数
    int nbx_[N+1];//保存每个时刻状态量约束的个数
    int nbu_[N+1];//保存每个时刻控制量约束的个数
    int ng_[N+1];//保存每个时刻多体约束的数量
    int nsbx_[N+1];//保存每个时刻关于x的松弛系数的个数
    int nsbu_[N+1];//保存每个时刻关于u的松弛系数的个数
    int nsg_[N+1];//保存每个时刻关于polytopic constraints的松弛系数的个数

    // LTV dynamics
    // x_k+1 = A_k x_k + B_k u_k + b_k
    double *hA_[N]; //hA[k] = A_k
    double *hB_[N]; //hB[k] = B_k
    double *hb_[N]; //hb[k] = b_k

    // Cost (without soft constraints)
    // min_x,u sum_k=0^N 1/2*[x_k;u_k]^T*[Q_k , S_k; S_k^T , R_k]*[x_k;u_k] + [q_k; r_k]^T*[x_k;u_k]
    double *hQ_[N+1]; //hQ[k] = Q_k
    double *hS_[N+1]; //hS[k] = S_k
    double *hR_[N+1]; //hR[k] = R_k
    double *hq_[N+1]; //hq[k] = q_k
    double *hr_[N+1]; //hr[k] = r_k

    // Polytopic constraints
    // g_lower,k <= D_k*x_k + C_k*u_k
    // D_k*x_k + C_k*u_k  <= g_upper,k
    double *hlg_[N+1]; //hlg[k] =  g_lower,k
    double *hug_[N+1]; //hug[k] =  g_upper,k
    double *hC_[N+1]; //hC[k] = C_k
    double *hD_[N+1]; //hD[k] = D_k

    // General bounds
    // x_lower,k <= x_k <= x_upper,k
    int *hidxbx_[N+1]; // 储存每个时刻状态量约束的数量，hidxbx[k] = {0,1,2,...,nx} for bounds on all inputs and states
    double *hlbx_[N+1]; // 储存每个时刻状态量下限的首地址 x_lower,k
    double *hubx_[N+1]; //储存每个时刻状态量上限的首地址x_upper,k
    // u_lower,k <= u_k <=  u_upper,k
    int *hidxbu_[N+1]; // 储存每个时刻控制量约束的数量，hidxbuk] = {0,1,2,...,nu} for bounds on all inputs and states
    double *hlbu_[N+1]; // 储存每个时刻控制量下限的首地址 u_lower,k
    double *hubu_[N+1]; // 储存每个时刻控制量上限的首地址u_upper,k

    // Cost (only soft constriants)
    // s_lower,k -> slack variable of lower polytopic constraint (3) + lower bounds
    // s_upper,k -> slack variable of upper polytopic constraint (4) + upper bounds
    // min_x,u sum_k=0^N 1/2*[s_lower,k;s_upper,k]^T*[Z_lower,k , 0; 0 , Z_upper,k]*[s_lower,k;s_upper,k] + [z_lower,k; z_upper,k]^T*[s_lower,k;s_upper,k]
    double *hZl_[N+1]; // hZl[k] = Z_lower,k
    double *hZu_[N+1]; // hZu[k] = Z_upper,k
    double *hzl_[N+1]; // hzl[k] = z_lower,k
    double *hzu_[N+1]; // hzu[k] = z_upper,k

    // 软约束乘子的边界
    double *hlls_[N+1];//储存每个时刻软约束下界的首地址
    double *hlus_[N+1];//储存每个时刻软约束上界的首地址
    int *hidxs_[N+1];//储存每个时刻边界和约束被软化的个数

    //HpipmBound 和 stages bounds 不同，需要储存在指针可以指向的地方
    std::array<HpipmBound,N+1> hpipm_bounds_;//将stages转化为std::vector格式的结构
    Eigen::Matrix<double,NX,1> b0_;//赋值给hb_的指针
    //设置动力学参数
    void setDynamics(std::array<Stage,N+1> &stages,const State &x0);
    //设置代价
    void setCost(std::array<Stage,N+1> &stages);
    //设置边界条件
    void setBounds(std::array<Stage,N+1> &stages,const State &x0);
    //设置约束 g = cu + dx
    void setPolytopicConstraints(std::array<Stage,N+1> &stages);
    //设置软约束
    void setSoftConstraints(std::array<Stage,N+1> &stages);
    //进行求解
    std::array<OptVariables,N+1> Solve(int *status);

    void print_data();
};
}
