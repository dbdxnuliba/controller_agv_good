#include "adrc_cascade.h"

namespace adrc
{
adrc_cascade::adrc_cascade(const double &h, const double &r)
    : adrc_base(h,r)
    , state_outer_eso_z1_(0.0)
    , state_outer_eso_z2_(0.0)
    , state_outer_u_(0.0)
    , state_inner_eso_z1_(0.0)
    , state_inner_eso_z2_(0.0)
    , state_inner_u_(0.0)
    , para_b0_(1.0)
    , para_eso_beta1_(200.0)
    , para_eso_beta2_(20000.0)
    , para_outer_nlsef_k_(1.0)
    , para_outer_nlsef_alpha_(0.5)
    , para_outer_nlsef_delta_(0.5)
    , para_inner_nlsef_k_(10.0)
    , para_inner_nlsef_alpha_(0.5)
    , para_inner_nlsef_delta_(0.5)
{
}

adrc_cascade::~adrc_cascade()
{
}

void adrc_cascade::OuterESO(const double &y)
{
    double e = state_outer_eso_z1_ - y;
    state_outer_eso_z1_ += para_h_ * (state_outer_eso_z2_ - para_eso_beta1_ * e + state_outer_u_);
    state_outer_eso_z2_ += para_h_ * (-para_eso_beta2_ * e);
}

void adrc_cascade::OuterNLSEF(const double &y)
{
    double e = state_td_v1_ - y;
    state_outer_u_ = para_outer_nlsef_k_ * e;
}

void adrc_cascade::InnerESO(const double &y)
{
    double e = state_inner_eso_z1_ - y;
    state_inner_eso_z1_ += para_h_ * (state_inner_eso_z2_ - para_eso_beta1_ * e + para_b0_ *state_inner_u_);
    state_inner_eso_z2_ += para_h_ * (-para_eso_beta2_ * e);
}

void adrc_cascade::InnerNLSEF()
{
    double e = state_outer_u_ - state_inner_eso_z1_;
    state_inner_u_ = para_inner_nlsef_k_ * e;
    state_inner_u_ -= state_inner_eso_z2_;
    state_inner_u_ /= para_b0_;
}

double adrc_cascade::CascadeControl(const double &ref, const double &outer_cur, const double &inner_cur)
{
    TD(ref);
    OuterNLSEF(outer_cur);
    InnerESO(inner_cur);
    InnerNLSEF();

    ConstraintControl(state_inner_u_);
    return state_inner_u_;
}
void adrc_cascade::PrintParameters(std::ostream &os) const
{
    adrc_base::PrintParameters(os);
    os << "b0: " << para_b0_ << " ";
    os << "eso beta1: " << para_eso_beta1_ << " beta2: " << para_eso_beta2_ << std::endl;
    os << "inner_nlsef k: " << para_outer_nlsef_k_ << " alpha: " << para_outer_nlsef_alpha_ << " delta： " << para_outer_nlsef_delta_ << std::endl;
    os << "inner_nlsef k: " << para_inner_nlsef_k_ << " alpha: " << para_inner_nlsef_alpha_ << " delta： " << para_inner_nlsef_delta_ << std::endl;
}
} // namespace adrc
