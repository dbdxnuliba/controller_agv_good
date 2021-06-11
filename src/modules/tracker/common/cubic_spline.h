/*************************************************************************
  > File Name: cubic_spline.h
  > Author: pan.cao
  > Mail: umrobot
  > Created Time:  8 27 2020
 ************************************************************************/
#pragma once
#include<iostream>
#include<vector>
#include<array>
#include<string>
#include<Eigen/Eigen>
#include<stdexcept>
#include"data_type.h"

namespace bz_robot{

class Spline{
public:

  Spline();
  Spline(Vec_f x, Vec_f y);

  float CalculatePositionFromS(float s);
  float CalculateDrivate(float s);
  float CalculateSecondDrivate(float s);

private:
  Eigen::MatrixXf CalculateA();
  Eigen::VectorXf CalculateB();
  //二分法
  int Bisect(float t, int start, int end);
private:
  Vec_f x_;
  Vec_f y_;
  Vec_f h_;
  Vec_f a_;
  Vec_f b_;
  Vec_f c_;
  Vec_f d_;
  int   nx_;
};
}
