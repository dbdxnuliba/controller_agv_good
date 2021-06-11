#include "adrc_firstorder.h"

namespace adrc
{

adrc_firstorder::adrc_firstorder(const double &h, const double &r)
    : adrc_base(h,r)
    , state_leso_z1_(0.0)
    , state_leso_z2_(0.0)
    , state_cur_u_(0.0)
    , para_b0_(1.0)
    , para_leso_beta1_(200.0)
    , para_leso_beta2_(13333.0)
    , para_lsef_k1_(10.0)
    , para_lsef_k2_(10.0)
{
}

adrc_firstorder::~adrc_firstorder()
{
}
//一阶线性状态观测器
void adrc_firstorder::LESO(const double &y)
{
    double e = state_leso_z1_ - y;
    state_leso_z1_ += para_h_ * (state_leso_z2_ - para_leso_beta1_ * e + para_b0_ * state_cur_u_);
    state_leso_z2_ += para_h_ * (-para_leso_beta2_ * e);
}
//一阶线性反馈
void adrc_firstorder::LSEF(const double &ref, const double &cur)
{
    double e = ref - state_leso_z1_;
    state_cur_u_ = para_lsef_k1_ * e;
    state_cur_u_ -= state_leso_z2_;
    state_cur_u_ /= para_b0_;
}
double adrc_firstorder::ComputeControl(const double &ref, const double &cur)
{
    LESO(cur);
    LSEF(ref, cur);
    ConstraintControl(state_cur_u_);
    return state_cur_u_;
}

void adrc_firstorder::PrintParameters(std::ostream &os) const
{
    adrc_base::PrintParameters(os);
    os << "LESO: beta1: " << para_leso_beta1_ << " beta2: " << para_leso_beta2_ << std::endl;
    os << "LSEF: k1: " << para_lsef_k1_ << " k2: " << para_lsef_k2_ << " b0: " << para_b0_ << std::endl;
    os<<std::endl;
}
} // namespace adrc
