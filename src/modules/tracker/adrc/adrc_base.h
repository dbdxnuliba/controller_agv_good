#pragma once
#include <math.h>
#include <iostream>
#include <float.h>
namespace adrc
{
class adrc_base
{
public:
  adrc_base(const double &h,const double &r);
  virtual ~adrc_base();
  int Sign(double val);
  double Fal(const double &error, const double &alpha, const double &delta);
  double Fhan(const double &x1, const double &x2, const double &r, const double &h);
  void ConstraintControl(double &u);
  void SetLimit(const double &limit_up, const double &limit_down);
  //跟踪微分器
  virtual void TD(const double &v0);
  virtual double ComputeControl(const double &ref, const double &cur);
  virtual void PrintParameters(std::ostream &os) const;
public:
  double para_h_;
  double para_td_r_;
protected:
  double state_td_v1_;
  double state_td_v2_;
  double limit_up_;
  double limit_down_;
};
std::ostream &operator<<(std::ostream &os, const adrc_base &item);
}
