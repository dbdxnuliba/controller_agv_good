#ifndef ADRC_SECONDORDER_H
#define ADRC_SECONDORDER_H
#include "adrc_base.h"
#include <memory>
namespace adrc
{

enum sef_mode_t
{
    LSEF,               //u0 = k1*e1+k2*e2
    NLSEF_1,            //u0 = k1*fal(e1,alpa1,deta)+k1*fal(e2,alpa2,deta)
    NLSEF_2,            //u0 = -fhan(e1,e2,r,h1)
    NLSEF_3             //u0 = -fhan(e1,c*e2,r,h1)
};

enum eso_mode_t
{
    LESO,
    NLESO
};

typedef struct para_eso
{
    eso_mode_t mode;
    double beta1;
    double beta2;
    double beta3;
    double alpha1;
    double alpha2;
    double delta;
} para_eso_t;

typedef struct para_sef
{
    sef_mode_t mode;
    double k1;
    double k2;
    double alpha1;
    double alpha2;
    double delta;
    double r;
    double c;
    double h1;
} para_sef_t;
class adrc_secondorder;
using adrc_secondorderHdr = std::shared_ptr<adrc_secondorder>;
class adrc_secondorder : public adrc_base
{

public:
    adrc_secondorder(const double &h,const double &r);

    ~adrc_secondorder();
    void ESO(const double &y);
    void SEF();
    virtual double ComputeControl(const double &ref, const double &cur);
    virtual void PrintParameters(std::ostream &os) const;
    //通用参数
    double para_b0_;
    //ESO参数
    para_eso_t para_eso_;
    //NLSEF参数
    para_sef_t para_sef_;

private:
    double state_eso_z1_;
    double state_eso_z2_;
    double state_eso_z3_;

    double state_cur_u_;
};

} // namespace adrc

#endif
