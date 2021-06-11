#pragma once
#include <iostream>
#include <string>
#include <math.h>
#include "deal_with_track.h"
#include "frenet_connect_cartesian.h"
#include <memory>
#include <angle.h>
#include "controller_to_ros.h"
#include "common/print.h"
//2-14-49-49-1 模式
#define MAXITER 1000   //最大迭代次数
#define BETA 0.35 //学习率
#define DIMINPUT 2 //输入样本的维数
#define DIMOUTPUT 1 // 输出样本的维数
#define DIMFUZZY  7 // 输入变量的论域大小
#define DIMRULE   49// 总的规则数
using floatvector = std::vector<float>;
namespace bz_robot
{
class Fuzzy_Neural_Network
{
public:
  Fuzzy_Neural_Network();
  ~Fuzzy_Neural_Network();
  void TrainNet(std::vector<floatvector> inputTrain, std::vector<floatvector> outputTrain);
private:
  std::vector<floatvector> GetInputTrain(char *File);  //获取训练样本中的输入
  std::vector<floatvector> GetOutputTrain(char *File);  //获取训练样本中的期望输出
  std::vector<floatvector> InputNormalization(std::vector<floatvector> inputTrain);  //训练样本归一化
  std::vector<floatvector> OutputNormalization(std::vector<floatvector> outputTrain);  //输出期望归一化
  float ComputeNormalDistribution(float u,float b,float x);  //u为正态分布均值，b为正态分布标准差，x为变量
};
}

