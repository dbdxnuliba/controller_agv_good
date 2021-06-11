#include "generate_random_number.h"
namespace bz_robot
{

GenerateRandom::GenerateRandom()
{

}

GenerateRandom::~GenerateRandom()
{

}

float GenerateRandom::GenerateGaussian(const float &mean, const float &variance)
{
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine gen(seed);
  std::normal_distribution<float> norm(mean,variance);
  float r= norm(gen);
  clamp(r,mean-variance-variance,mean+variance+variance);
  //std::cout<<"高斯正态分布生成："<<r<<std::endl;
  return r;
}

}
