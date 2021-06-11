#include "adrc_base.h"
namespace adrc
{

adrc_base::adrc_base(const double &h,const double &r)
{
  state_td_v1_ = 0.0;
  state_td_v2_ = 0.0;
  para_td_r_   = r;
  para_h_      = h;
  limit_up_    = 1e9;
  limit_down_  = -1e9;
}

adrc_base::~adrc_base()
{

}

int adrc_base::Sign(double val)
{
  if (val > DBL_EPSILON)
    return 1;
  else  if(val  < -DBL_EPSILON)
    return -1;
  else return 0;
}

double adrc_base::Fal(const double &error, const double &alpha, const double &delta)
{
  if (fabs(error) > delta)
  {
    return Sign(error) * pow(fabs(error), alpha);
  }
  else
  {
    return error / (pow(delta, 1.0f - alpha));
  }
}

double adrc_base::Fhan(const double &x1, const double &x2, const double &r, const double &h)
{
  double d = r * h * h;
  double a0 = h * x2;
  double y = x1 + a0;
  double a1 = sqrt(d * (d + 8.0f * fabs(y)));
  double a2 = a0 + Sign(y) * (a1 - d) * 0.5f;
  double sy = (Sign(y + d) - Sign(y - d)) * 0.5f;
  double a = (a0 + y - a2) * sy + a2;
  double sa = (Sign(a + d) - Sign(a - d)) * 0.5f;

  return -r * (a / d - Sign(a)) * sa - r * Sign(a);
}

void adrc_base::ConstraintControl(double &u)
{
  if (u > limit_up_)
  {
    u = limit_up_;
  }
  else if (u < limit_down_)
  {
    u = limit_down_;
  }
}

void adrc_base::SetLimit(const double &limit_up, const double &limit_down)
{
  limit_up_ = fabs(limit_up);
  limit_down_ = -fabs(limit_down);
}

void adrc_base::TD(const double &v0)
{
  double fh = Fhan(state_td_v1_ - v0, state_td_v2_, para_td_r_, 3*para_h_);
  state_td_v1_ += para_h_ * state_td_v2_;
  state_td_v2_ += para_h_ * fh;
  //std::cout<<"跟踪微分器v1:"<<state_td_v1_<<std::endl;
  //std::cout<<"跟踪微分器v2:"<<state_td_v2_<<std::endl;
}

double adrc_base::ComputeControl(const double &ref, const double &cur)
{
  TD(ref);
  ConstraintControl(state_td_v2_);
  return state_td_v2_;
}

void adrc_base::PrintParameters(std::ostream &os) const
{
  os << "control limit: " << limit_down_ << " - " << limit_up_ << std::endl;
  os << "TD: r: " << para_td_r_ << " h: " << para_h_ << std::endl;
}
std::ostream &operator<<(std::ostream &os, const adrc_base &obj)
{
  obj.PrintParameters(os);
  return os;
}
}
