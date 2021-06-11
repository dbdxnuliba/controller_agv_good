#include "fnn_controller.h"
namespace bz_robot
{
Fuzzy_Neural_Network::Fuzzy_Neural_Network()
{
}

Fuzzy_Neural_Network::~Fuzzy_Neural_Network()
{

}

void Fuzzy_Neural_Network::TrainNet(std::vector<floatvector> inputTrain, std::vector<floatvector> outputTrain)
{
  //需要优化的参数，主要包括
  int i, j, k;
  float p[DIMINPUT+1][DIMRULE], p_1[DIMINPUT+1][DIMRULE], d_p[DIMRULE];//一阶T-S系统输出系数
  float c[DIMFUZZY][DIMINPUT], c_1[DIMFUZZY][DIMINPUT];//高斯隶属度函数的均值，本次迭代及上次迭代的结果
  float b[DIMFUZZY][DIMINPUT], b_1[DIMFUZZY][DIMINPUT];//高斯隶属度函数的标准差，本次迭代及上次迭代的结果
  float u[DIMINPUT][DIMFUZZY];//每个输入，在每个论域下对应的隶属度值
  float w[DIMRULE];//计算每条规则的权重
  float y[DIMRULE];//根据T-S模型获得的规则输出

  //初始化参数，包括输入隶属度函数，以及输出T-S函数
  srand(time(NULL));
  for(i=0; i<DIMFUZZY; i++)
  {
    for(j=0; j<DIMINPUT; j++)
    {
      c[i][j] = ((float)rand()/RAND_MAX);//产生0-1之间的随机数
      b[i][j] = ((float)rand()/RAND_MAX);
      c_1[i][j] = c[i][j];
      b_1[i][j] = b[i][j];
    }
  }
  for(i=0; i<DIMINPUT+1; i++)
  {
    for(j=0; j<DIMRULE; j++)
    {
      p[i][j] = 0.18;
      p_1[i][j] = p[i][j];
    }
  }

  //开始训练
  float addw;//前置条件之和
  float addyw;//前置条件与规则对应中心值乘积之和
  float e;//理想输出与实际输出之差
  float yn;//实际输出

  for(int iter=0; iter<MAXITER; iter++)
  {
    for(k=0; k<inputTrain.size(); k++)
    {
      //获取输入变量的模糊隶属度，dimIn=4，hidePoint=8。
      for(i=0; i<DIMINPUT; i++)
        for(j=0; j<DIMFUZZY; j++)
          u[i][j] = exp(-((inputTrain[k][i]-c[j][i])*(inputTrain[k][i]-c[j][i]))/b[j][i]);

      addw = 0;
      for(i=0; i<DIMFUZZY; i++)
      {
        for(j=0; j<DIMFUZZY; j++)
        {
          w[i*DIMFUZZY+j] = u[0][i] * u[1][j];
          addw += w[i*DIMFUZZY+j];
        }
      }

      //计算输出
      addyw = 0;
      for(i=0; i<DIMRULE; i++)
      {
        float sumtemp =0;
        for(j=0; j<DIMINPUT; j++)
          sumtemp = sumtemp+p[j][i]*inputTrain[k][j];

        y[i] = sumtemp+p[DIMINPUT][i];

        addyw += y[i]*w[i];
      }

      yn = addyw/addw;    //模糊输出值

      e = outputTrain[k][0]-yn;

      //修正系数p
      for(i=0; i<DIMRULE; i++)
        d_p[i] = BETA*e*w[i]/addw;


      for(i=0; i<DIMRULE; i++)
      {
        for(j=0; j<DIMINPUT; j++)
          p[j][i] = p_1[j][i]+d_p[i]*inputTrain[k][j];
        p[DIMINPUT][i] = p_1[DIMINPUT][i];
      }
      //修正系数b
      for(i=0; i<DIMFUZZY; i++)
        for(j=0; j<DIMINPUT; j++)
          b[i][j] = b_1[i][j]+BETA*e*(y[i]*addw-addyw)*(inputTrain[k][j]-c[i][j])*(inputTrain[k][j]-c[i][j])*w[i]/(b[i][j]*b[i][j]*addw*addw);

      //输入隶属度函数的均值c ck+1 = ck + alpha*
      for(i=0; i<DIMFUZZY; i++)
        for(j=0; j<DIMINPUT; j++)
          c[i][j] = c_1[i][j]+BETA*e*(y[i]*addw-addyw)*2*(inputTrain[k][j]-c[i][j])*w[i]/(b[i][j]*addw*addw);

    }
  }

}

std::vector<floatvector> Fuzzy_Neural_Network::GetInputTrain(char *File)
{
  int i=1;
  floatvector temp;
  std::vector<floatvector> dst;
  float num;

  FILE *fp = fopen(File, "r");
  if(fp==NULL)
  {
    printf("Open input train error\n");
    exit(0);
  }

  temp.clear();
  while(fscanf(fp, "%lf", &num)!=EOF)
  {
    temp.push_back(num);
    if(i%DIMINPUT==0)
    {
      dst.push_back(temp);
      temp.clear();
    }
    i++;
  }
  return dst;
}

std::vector<floatvector> Fuzzy_Neural_Network::GetOutputTrain(char *File)
{
  int i=1;
  floatvector temp;
  std::vector<floatvector> dst;
  double num;

  FILE *fp = fopen(File, "r");

  if(fp==NULL)
  {
    printf("Open input train error\n");
    exit(0);
  }

  temp.clear();
  while(fscanf(fp, "%lf", &num)!=EOF)
  {
    temp.push_back(num);
    if(i%DIMOUTPUT==0)
    {
      dst.push_back(temp);
      temp.clear();
    }    i++;
  }
  return dst;
}

std::vector<floatvector> Fuzzy_Neural_Network::InputNormalization(std::vector<floatvector> inputTrain)
{
  float y;
  float maxinputtrain[DIMINPUT], mininputtrain[DIMINPUT];
  floatvector temp;
  std::vector<floatvector> dst;


  for(int i=0; i<DIMINPUT; i++)
  {
    maxinputtrain[i] = mininputtrain[i] = inputTrain[0][i];

    for(int j=1; j<inputTrain.size(); j++)
    {
      if(maxinputtrain[i]<inputTrain[j][i])
        maxinputtrain[i] = inputTrain[j][i];


      if(mininputtrain[i]>inputTrain[j][i])
        mininputtrain[i] = inputTrain[j][i];
    }
  }
  //归一化
  for(int i=0; i<inputTrain.size(); i++)
  {
    temp.clear();
    for(int j=0; j<DIMINPUT; j++)
    {
      y = (0.02+0.996*(inputTrain[i][j]-mininputtrain[j]))/(maxinputtrain[j]-mininputtrain[j]);
      temp.push_back(y);
    }
    dst.push_back(temp);
  }


  return dst;
}

std::vector<floatvector> Fuzzy_Neural_Network::OutputNormalization(std::vector<floatvector> outputTrain)
{
  float y;
  float maxoutputtrain[DIMOUTPUT], minoutputtrain[DIMOUTPUT];
  floatvector temp;
  std::vector<floatvector> dst;

  for(int i=0; i<DIMOUTPUT; i++)
  {
    maxoutputtrain[i] = minoutputtrain[i] = outputTrain[0][i];

    for(int j=1; j<outputTrain.size(); j++)
    {
      if(maxoutputtrain[i]<outputTrain[j][i])
        maxoutputtrain[i] = outputTrain[j][i];

      if(minoutputtrain[i]>outputTrain[j][i])
        minoutputtrain[i] = outputTrain[j][i];
    }
  }
  //归一化
  for(int i=0; i<outputTrain.size(); i++)
  {
    temp.clear();
    for(int j=0; j<DIMOUTPUT; j++)
    {
      y = (0.02+0.996*(outputTrain[i][j]-minoutputtrain[j]))/(maxoutputtrain[j]-minoutputtrain[j]);
      temp.push_back(y);
    }

    dst.push_back(temp);
  }

  return dst;
}

float Fuzzy_Neural_Network::ComputeNormalDistribution(float u, float b, float x)
{
  float y = exp(-((x-u)*(x-u))/(b*b));
  return y;
}
}

