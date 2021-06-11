#pragma once
#include "adrc_base.h"
namespace adrc
{
class adrc_cascade : public adrc_base
{

public:
  adrc_cascade(const double &h,const double &r);
  ~adrc_cascade();
  void OuterESO(const double &y);
  void OuterNLSEF(const double &y);
  void InnerESO(const double &y);
  void InnerNLSEF();

  double CascadeControl(const double &ref, const double &outer_cur, const double &inner_cur);
  virtual void PrintParameters(std::ostream &os) const;
  double para_b0_;

  double para_eso_beta1_;
  double para_eso_beta2_;

  double para_outer_nlsef_k_;
  double para_outer_nlsef_alpha_;
  double para_outer_nlsef_delta_;

  double para_inner_nlsef_k_;
  double para_inner_nlsef_alpha_;
  double para_inner_nlsef_delta_;

private:
  /* data */
  double state_outer_eso_z1_;
  double state_outer_eso_z2_;
  double state_outer_u_;

  double state_inner_eso_z1_;
  double state_inner_eso_z2_;
  double state_inner_u_;
};

} // namespace adrc
