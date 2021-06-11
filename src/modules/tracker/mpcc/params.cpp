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

#include "params.h"
namespace mpcc{
    
Param::Param(){
    std::cout << "Default initialization of model params" << std::endl;
}

Param::Param(std::string file)
{
  std::ifstream iModel(file);
  if(iModel.is_open())
  {
    std::cout<<"connect model json success"<<std::endl;
  }
  std::stringstream  oss;
  oss << iModel.rdbuf();
  //std::cout << oss.str() << std::endl;
  nlohmann::json jsonModel = nlohmann::json::parse(oss.str());
  // Model Parameters
  Cm1 	= jsonModel["Cm1"];
  Cm2 	= jsonModel["Cm2"];

  Cr0 	= jsonModel["Cr0"];
  Cr2 	= jsonModel["Cr2"];

  Br 	= jsonModel["Br"];
  Cr 	= jsonModel["Cr"];
  Dr 	= jsonModel["Dr"];

  Bf 	= jsonModel["Bf"];
  Cf 	= jsonModel["Cf"];
  Df 	= jsonModel["Df"];

  m 	= jsonModel["m"];
  Iz 	= jsonModel["Iz"];
  lf 	= jsonModel["lf"];
  lr 	= jsonModel["lr"];

  car_l = jsonModel["car_l"];
  car_w = jsonModel["car_w"];

  g = jsonModel["g"];
  //Constraint Parameters
  r_in = jsonModel["R_in"];
  r_out = jsonModel["R_out"];

  max_dist_proj = jsonModel["max_dist_proj"];

  e_long = jsonModel["E_long"];
  e_eps = jsonModel["E_eps"];

  max_alpha = jsonModel["maxAlpha"];
  // initial warm start and trust region (model dependent)
  initial_velocity = jsonModel["initial_velocity"];
  s_trust_region = jsonModel["s_trust_region"];

  vx_zero = jsonModel["vx_zero"];
}

CostParam::CostParam(){
    std::cout << "Default initialization of cost" << std::endl;
}

CostParam::CostParam(std::string file)
{
  std::cout << file << std::endl;
  std::ifstream iModel(file);
  if(iModel.is_open())
  {
    std::cout<<"connect cost json success"<<std::endl;
  }
  std::stringstream  oss;
  oss << iModel.rdbuf();
  //std::cout << oss.str() << std::endl;
  nlohmann::json jsonCost = nlohmann::json::parse(oss.str());

  q_c = jsonCost["qC"];
  q_l = jsonCost["qL"];
  q_vs = jsonCost["qVs"];

  q_mu = jsonCost["qMu"];

  q_r = jsonCost["qR"];

  q_beta = jsonCost["qBeta"];
  beta_kin_cost = 1;//jsonCost["betaKin"];

  r_D = jsonCost["rD"];
  r_delta = jsonCost["rDelta"];
  r_vs = jsonCost["rVs"];

  r_dD = jsonCost["rdD"];
  r_dDelta = jsonCost["rdDelta"];
  r_dVs = jsonCost["rdVs"];

  q_c_N_mult = jsonCost["qCNmult"];
  q_r_N_mult = jsonCost["qRNmult"];

  sc_quad_track = jsonCost["sc_quad_track"];
  sc_quad_tire= jsonCost["sc_quad_tire"];
  sc_quad_alpha = jsonCost["sc_quad_alpha"];

  sc_lin_track = jsonCost["sc_lin_track"];
  sc_lin_tire = jsonCost["sc_lin_tire"];
  sc_lin_alpha = jsonCost["sc_lin_alpha"];
}

BoundsParam::BoundsParam() {
    std::cout << "Default initialization of bounds" << std::endl;
}

BoundsParam::BoundsParam(std::string file)
{
  std::cout << file << std::endl;
  std::ifstream iModel(file);
  if(iModel.is_open())
  {
    std::cout<<"connect bound json success"<<std::endl;
  }
  std::stringstream  oss;
  oss << iModel.rdbuf();
  //std::cout << oss.str() << std::endl;
  nlohmann::json jsonBounds = nlohmann::json::parse(oss.str());

  lower_state_bounds.X_l = jsonBounds["Xl"];
  lower_state_bounds.Y_l = jsonBounds["Yl"];
  lower_state_bounds.phi_l = jsonBounds["phil"];
  lower_state_bounds.vx_l = jsonBounds["vxl"];
  lower_state_bounds.vy_l = jsonBounds["vyl"];
  lower_state_bounds.r_l = jsonBounds["rl"];
  lower_state_bounds.s_l = jsonBounds["sl"];
  lower_state_bounds.D_l = jsonBounds["Dl"];
  lower_state_bounds.delta_l = jsonBounds["deltal"];
  lower_state_bounds.vs_l = jsonBounds["vsl"];

  upper_state_bounds.X_u = jsonBounds["Xu"];
  upper_state_bounds.Y_u = jsonBounds["Yu"];
  upper_state_bounds.phi_u = jsonBounds["phiu"];
  upper_state_bounds.vx_u = jsonBounds["vxu"];
  upper_state_bounds.vy_u = jsonBounds["vyu"];
  upper_state_bounds.r_u = jsonBounds["ru"];
  upper_state_bounds.s_u = jsonBounds["su"];
  upper_state_bounds.D_u = jsonBounds["Du"];
  upper_state_bounds.delta_u = jsonBounds["deltau"];
  upper_state_bounds.vs_u = jsonBounds["vsu"];

  lower_input_bounds.dD_l = jsonBounds["dDl"];
  lower_input_bounds.dDelta_l = jsonBounds["dDeltal"];
  lower_input_bounds.dVs_l = jsonBounds["dVsl"];

  upper_input_bounds.dD_u = jsonBounds["dDu"];
  upper_input_bounds.dDelta_u = jsonBounds["dDeltau"];
  upper_input_bounds.dVs_u = jsonBounds["dVsu"];
}

NormalizationParam::NormalizationParam(){
    std::cout << "Default initialization of normalization" << std::endl;
}

NormalizationParam::NormalizationParam(std::string file)
{
  std::cout << file << std::endl;
  std::ifstream iModel(file);
  if(iModel.is_open())
  {
    std::cout<<"connect normalization json success"<<std::endl;
  }
  std::stringstream  oss;
  oss << iModel.rdbuf();
  //std::cout << oss.str() << std::endl;
  nlohmann::json jsonNorm = nlohmann::json::parse(oss.str());
  //对角阵T_x
  T_x.setIdentity();
  T_x(si_index.X,si_index.X) = jsonNorm["X"];
  T_x(si_index.Y,si_index.Y) = jsonNorm["Y"];
  T_x(si_index.phi,si_index.phi) = jsonNorm["phi"];
  T_x(si_index.vx,si_index.vx) = jsonNorm["vx"];
  T_x(si_index.vy,si_index.vy) = jsonNorm["vy"];
  T_x(si_index.r,si_index.r) = jsonNorm["r"];
  T_x(si_index.s,si_index.s) = jsonNorm["s"];
  T_x(si_index.D,si_index.D) = jsonNorm["D"];
  T_x(si_index.delta,si_index.delta) = jsonNorm["delta"];
  T_x(si_index.vs,si_index.vs) = jsonNorm["vs"];
  //对角阵T_x的逆
  T_x_inv.setIdentity();
  for(int i = 0;i<NX;i++)
  {
    T_x_inv(i,i) = 1.0/T_x(i,i);
  }
  //对角阵T_u
  T_u.setIdentity();
  T_u(si_index.dD,si_index.dD) = jsonNorm["dD"];
  T_u(si_index.dDelta,si_index.dDelta) = jsonNorm["dDelta"];
  T_u(si_index.dVs,si_index.dVs) = jsonNorm["dVs"];
  //对角阵T_u的逆
  T_u_inv.setIdentity();
  for(int i = 0;i<NU;i++)
  {
    T_u_inv(i,i) = 1.0/T_u(i,i);
  }
  //对角阵T_u及它的逆
  T_s.setIdentity();
  T_s_inv.setIdentity();
}

}
