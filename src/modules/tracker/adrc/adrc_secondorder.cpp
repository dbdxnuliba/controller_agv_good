#include "adrc_secondorder.h"
namespace adrc
{
adrc_secondorder::adrc_secondorder(const double &h, const double &r)
  : adrc_base(h,r)
  , state_eso_z1_(0.0)
  , state_eso_z2_(0.0)
  , state_eso_z3_(0.0)
  , state_cur_u_(0.0)
  , para_b0_(1.0)
{
  //初始化ESO
  para_eso_.mode   = LESO;
  para_eso_.beta1  = 200.0;
  para_eso_.beta2  = 1768.0;
  para_eso_.beta3  = 13420.0;
//  para_eso_.mode   = NLESO;
//  para_eso_.beta1  = 200.0;
//  para_eso_.beta2  = 1768.0;
//  para_eso_.beta3  = 13420.0;
//  para_eso_.alpha1 = 0.5;
//  para_eso_.alpha2 = 0.25;
//  para_eso_.delta  = 0.05;
  //初始化SEF
  para_sef_.mode   = LSEF;
  para_sef_.k1     = 10.0;
  para_sef_.k2     = 10.0;
//  para_sef_.mode   = NLSEF_1;
//  para_sef_.k1     = 10.0;
//  para_sef_.k2     = 10.0;
//  para_sef_.alpha1 = 10.0;
//  para_sef_.alpha2 = 1.25;
//  para_sef_.delta  = 0.5;
//  para_sef_.r      = 20000.0;
//  para_sef_.c      = 0.5;
//  para_sef_.h1     = 0.025;
}

adrc_secondorder::~adrc_secondorder()
{}

void adrc_secondorder::ESO(const double &y)
{
  double e = state_eso_z1_ - y;
  state_eso_z1_ += para_h_ * (state_eso_z2_ - para_eso_.beta1 * e);
  std::cout<<"LESO:state_eso_z1_:"<<state_eso_z1_<<std::endl;
  std::cout<<"LESO:y:"<<y<<std::endl;
  switch (para_eso_.mode)
  {
    case LESO:
    {
      state_eso_z2_ += para_h_ * (state_eso_z3_ - para_eso_.beta2 * e + para_b0_ * state_cur_u_);
      state_eso_z3_ += para_h_ * (-para_eso_.beta3 * e);
    }
      std::cout<<"LESO:state_eso_z1_:"<<state_eso_z1_<<std::endl;
      std::cout<<"LESO:state_eso_z2_:"<<state_eso_z2_<<std::endl;
      std::cout<<"LESO:state_eso_z3_:"<<state_eso_z3_<<std::endl;
      break;
    case NLESO:
    {
      double fe = Fal(e, para_eso_.alpha1, para_eso_.delta);
      double fe1 = Fal(e, para_eso_.alpha2, para_eso_.delta);
      state_eso_z2_ += para_h_ * (state_eso_z3_ - para_eso_.beta2 * fe + para_b0_ * state_cur_u_);
      state_eso_z3_ += para_h_ * (-para_eso_.beta3 * fe1);
    }
      std::cout<<"NLESO:state_eso_z1_:"<<state_eso_z1_<<std::endl;
      std::cout<<"NLESO:state_eso_z2_:"<<state_eso_z2_<<std::endl;
      std::cout<<"NLESO:state_eso_z3_:"<<state_eso_z3_<<std::endl;
      break;
    default:
      break;
  }
}

void adrc_secondorder::SEF()
{
  double e1 = state_td_v1_ - state_eso_z1_;
  double e2 = state_td_v2_ - state_eso_z2_;

  switch (para_sef_.mode)
  {
    case LSEF:
      state_cur_u_ = para_sef_.k1 * e1 + para_sef_.k2 * e2;
      break;
    case NLSEF_1:
      state_cur_u_ = para_sef_.k1*Fal(e1,para_sef_.alpha1,para_sef_.delta)
          + para_sef_.k2*Fal(e2,para_sef_.alpha2,para_sef_.delta);
      break;
    case NLSEF_2:
      state_cur_u_ = -Fhan(e1, e2, para_sef_.r, para_sef_.h1);
      break;
    case NLSEF_3:
      state_cur_u_ = -Fhan(e1, para_sef_.c * e2, para_sef_.r, para_sef_.h1);
      break;

    default:
      break;
  }

  state_cur_u_ -= state_eso_z3_;
  state_cur_u_ /= para_b0_;
}

double adrc_secondorder::ComputeControl(const double &ref, const double &cur)
{
  TD(ref);
  ESO(cur);
  SEF();
  ConstraintControl(state_cur_u_);
  return state_cur_u_;
}

void adrc_secondorder::PrintParameters(std::ostream &os) const
{
  adrc_base::PrintParameters(os);
  os << "ESO: "<< "beta1: " << para_eso_.beta1 << " beta2: " << para_eso_.beta2 << " beta3: " << para_eso_.beta3 << std::endl;

  if(para_eso_.mode == NLESO)
  {
    os << "alpha1: "<<para_eso_.alpha1<<" alpha2: "<<para_eso_.alpha2<<" delta"<<para_eso_.delta << std::endl;
  }
  os << "SEF: b0: " <<para_b0_<< std::endl;

  switch (para_sef_.mode)
  {
    case LSEF:
      os << "k1: " << para_sef_.k1 <<" k2:"<<para_sef_.k2<< std::endl;
      break;
    case NLSEF_1:
      os << "k1: " << para_sef_.k1 << " k2:" << para_sef_.k2 << std::endl;
      os << "alpha1: " << para_sef_.alpha1 << " alpha2: " << para_sef_.alpha2 << " delta" << para_sef_.delta << std::endl;
      break;
    case NLSEF_2:
      os << "r1: " << para_sef_.r << " h1: " << para_sef_.h1 << std::endl;
      break;
    case NLSEF_3:
      os << "r1: " << para_sef_.r << " c: " << para_sef_.c << " h1: " << para_sef_.h1 << std::endl;
      break;

    default:
      break;
  }
}
} // namespace adrc
