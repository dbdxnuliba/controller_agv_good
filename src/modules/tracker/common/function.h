#pragma once
class function
{
public:
  function();
  ~function();
  //u为正态分布均值，b为正态分布标准差，x为变量
  float ComputeNormalDistribution(float u,float b,float x);
};
