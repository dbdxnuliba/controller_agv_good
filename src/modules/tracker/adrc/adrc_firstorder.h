#pragma once
#include "adrc_base.h"
namespace adrc
{

class adrc_firstorder : public adrc_base
{

public:
    adrc_firstorder(const double &h,const double &r);

    ~adrc_firstorder();
    // virtual void TD(const double &v0);
    void LESO(const double &y);
    void LSEF(const double &ref, const double &cur);
    virtual double ComputeControl(const double &ref, const double &cur);

    virtual void PrintParameters(std::ostream &os) const;
    double GetLESOZ1()
    {
        return state_leso_z1_;
    }

    double GetLESOZ2()
    {
        return state_leso_z2_;
    }

    /**************LESO参数*************/
    double para_b0_;
    double para_leso_beta1_;
    double para_leso_beta2_;
    /**************LSEF参数*************/
    double para_lsef_k1_;
    double para_lsef_k2_;

private:
    double state_leso_z1_;
    double state_leso_z2_;
    double state_cur_u_;
};

} // namespace adrc
