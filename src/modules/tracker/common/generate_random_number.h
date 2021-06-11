#pragma once
#include <iostream>
#include <random>
#include <chrono>
#include <memory>
namespace bz_robot
{
class GenerateRandom;
using GenerateRandomHdr = std::shared_ptr<GenerateRandom>;
using GenerateRandomPtr = GenerateRandom*;
class GenerateRandom
{
public:
  GenerateRandom();
  ~GenerateRandom();
  static GenerateRandomPtr GetInstance()
  {
    static GenerateRandom instance;
    return &instance;
  }
  float GenerateGaussian(const float &mean, const float &variance);

  template <typename T>
  void inline clamp(T &x,const T &x_min,const T &x_max)
  {
    if(x<x_min)
    {
      x = x_min;
    }
    else if(x>x_max)
    {
      x = x_max;
    }
    else
    {
      x = x;
    }
  }
private:

};
}
