// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "arc_length_spline.h"
#include <float.h>
#include <angles/angles.h>
namespace mpcc{
ArcLengthSpline::ArcLengthSpline()
{ 
}
ArcLengthSpline::ArcLengthSpline(const PathToJson &path)
:param_(Param(path.param_path))
{
  target_point_index_ = 65535;
  target_point_.heading_angle = DBL_MAX;
  target_point_.position.x    = DBL_MAX;
  target_point_.position.y    = DBL_MAX;
}

double ArcLengthSpline::ComputeCurvature(const double &s)
{
  Eigen::Vector2d d = getDerivative(s);
  Eigen::Vector2d dd = getSecondDerivative(s);
  return (dd(1) * d(0) - dd(0) * d(1))/(d(0) * d(0) + d(1) * d(1));
}

//void ArcLengthSpline::setData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in)
//{
//    // set input data if x and y have same length
//    // compute arc length based on an piecewise linear approximation
//    if(X_in.size() == Y_in.size()){
//        path_data_.X = X_in;
//        path_data_.Y = Y_in;
//        path_data_.n_points = X_in.size();
//        path_data_.s = compArcLength(X_in,Y_in);
//    }
//    else{
//        std::cout << "input data does not have the same length" << std::endl;
//    }
//}
//设置路径x,y,s
void ArcLengthSpline::setRegularData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in,const Eigen::VectorXd &s_in)
{
  if(X_in.size() == Y_in.size())
  {
    //todo 先清理下
    //
    path_data_.X = X_in;
    path_data_.Y = Y_in;
    path_data_.n_points = X_in.size();
    path_data_.s = s_in;
    //std::cout<<"path_data_.X.size()="<<path_data_.X.size()<<std::endl;
    //std::cout<<"X_in.size()="<<X_in.size()<<std::endl;
  }
  else
  {
    std::cout << "input data does not have the same length" << std::endl;
  }
}
//获取前i个点对应的总弧长
Eigen::VectorXd ArcLengthSpline::compArcLength(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in) const
{
    //compute arc length based on straight line distance between the data points
  double dx,dy;
  double dist;

  int n_points = X_in.size();
  // initailize s as zero
  Eigen::VectorXd s;
  s.setZero(n_points);
  for(int i=0;i<n_points-1;i++)
  {
    dx = X_in(i+1)-X_in(i);
    dy = Y_in(i+1)-Y_in(i);
    dist = std::sqrt(dx*dx + dy*dy);//dist is straight line distance between points
    s(i+1) = s(i)+dist;//s is cumulative sum of dist
  }
  return s;
}
//对路径进行重采样，给定新路径的尺寸，保证每段路径等弧长，最终输出点坐标
PathData ArcLengthSpline::resamplePath(const CubicSpline &initial_spline_x,const CubicSpline &initial_spline_y,const double total_arc_length) const
{
    //resampled_path.n_points的大小不应该是定值,可以根据总的弧长进行动态调整,暂定单段弧长为0.1m
    PathData resampled_path;
    const int index = std::ceil(total_arc_length / 0.02);
    resampled_path.n_points=index;
    //将总弧长分解为等长度setLinSpaced(size,low,high)
    resampled_path.s.setLinSpaced(index,0,total_arc_length);
    resampled_path.s(index - 1) = total_arc_length;
    //初始化向量X，Y，长度为N_SPLINE
    resampled_path.X.setZero(index);
    resampled_path.Y.setZero(index);
    //根据弧长获取对应点的坐标
    for(int i=0;i<index;i++)
    {
        resampled_path.X(i) = initial_spline_x.getPoint(resampled_path.s(i));
        resampled_path.Y(i) = initial_spline_y.getPoint(resampled_path.s(i));
    }
    return resampled_path;
}
//去除部分异常的离群点
RawPath ArcLengthSpline::outlierRemoval(const Eigen::VectorXd &X_original,const Eigen::VectorXd &Y_original) const
{
    // 去除与其他点完全不等距的点。避免匹配问题
    //计算点之间的平均距离，去除距离小于0.7倍平均距离的点
    double dx,dy;       // 点之间的x和y偏差
    Eigen::VectorXd distVec;   // 弧长的容器
    double meanDist;    // 平均距离
    double dist;        // 点之间的距离
    RawPath resampled_path;//重采样后的路径信息
    int k = 0;          // 索引号，为经过一次重采样后的路径尺寸
    int j = 0;          // 索引号，为重采样过程中的过渡编号
    //防止出现错误的输入
    if (X_original.size() != Y_original.size()){
        //error
    }

    int n_points = X_original.size();
    // 初始化存放路径信息的容器
    resampled_path.X.setZero(n_points);
    resampled_path.Y.setZero(n_points);
    // 计算每一小段路径的原始长度
    distVec.setZero(n_points-1);
    for(int i=0;i<n_points-1;i++){
        dx = X_original(i+1)-X_original(i);
        dy = Y_original(i+1)-Y_original(i);
        distVec(i) = std::sqrt(dx*dx + dy*dy);
    }
    // 计算出路径的平均长度
    meanDist = distVec.sum()/(n_points-1);
    // 重采样，计算新的路径信息
    resampled_path.X(k) = X_original(k);
    resampled_path.Y(k) = Y_original(k);
    k++;
    for(int i=1;i<n_points-1;i++){
        // compute distance between currently checked point and the one last added to the new X-Y path
        dx = X_original(i)-X_original(j);
        dy = Y_original(i)-Y_original(j);
        dist = std::sqrt(dx*dx + dy*dy);
        // 如果距离大于0.7倍的平均距离， 将该点加入新路径中
        if(dist >= 0.7*meanDist)
        {
            resampled_path.X(k) = X_original(i);
            resampled_path.Y(k) = Y_original(i);
            k++;
            j = i;//就是为了记录上一个可以加入新路径的点的原有索引号
        }
    }
    // 将原有路径的最后一个点加入到新路径中
    resampled_path.X(k) = X_original(n_points-1);
    resampled_path.Y(k) = Y_original(n_points-1);
    k++;
    // 矩阵转维可以通过resize()函数得到。转维时，原来的元素值可能会改变，
    //如果希望保持原矩阵的值不变，可用conservativeResize()
    resampled_path.X.conservativeResize(k);
    resampled_path.Y.conservativeResize(k);
    //将重采样的路径输出
    return resampled_path;
}

double ArcLengthSpline::unwrapInput(double x) const
{
    double x_unwrap;
    //获取最终点的x值
    //std::cout << spline_data_.x_data.size() << " - " << spline_data_.n_points-1 << std::endl;
    double x_max = getLength();
    //todo作用是什么？？？？？？？？？？？
    //当x >= xmax 说明已经到达终点，返回调整后的弧长为0
    if(x > x_max)
      //TODO x_unwrap = 0;
      x_unwrap = x - x_max*std::floor(x/x_max);
    else if(x < 0)
      x_unwrap = 0;
    else
      x_unwrap = x;
    return x_unwrap;
}

RawPath ArcLengthSpline::extendPath(const RawPath &path)
{
  if(path.X.size() < 2)
  {
    std::cout<<"点太少,暂时不需要处理"<<std::endl;
    return path;
  }
  if(path.X.size() == 2)
  {
    double theta;
    double x0,y0,x1,y1,x2,y2;
    int size = path.X.size();
    RawPath path1;
    path1.X.resize(size + 1);
    path1.Y.resize(size + 1);

    x1 = path.X(size-1);
    y1 = path.Y(size-1);

    x2 = path.X(size-2);
    y2 = path.Y(size-2);

    theta = atan2(y1 -y2,x1 -x2);

    x0     = x1 + 1.5*std::cos(theta);
    y0     = y1 + 1.5*std::sin(theta);

    for(int i=0;i<size;i++)
    {
      path1.X(i) = path.X(i);
      path1.Y(i) = path.Y(i);
    }
    path1.X(size) = x0;
    path1.Y(size) = y0;

    return path1;
  }
  else
  {
    double theta,theta1,theta2;
    double x0,y0,x1,y1,x2,y2,x3,y3;
    int size = path.X.size();
    RawPath path1;
    path1.X.resize(size + 1);
    path1.Y.resize(size + 1);

    x1 = path.X(size-1);
    y1 = path.Y(size-1);

    x2 = path.X(size-2);
    y2 = path.Y(size-2);

    x3 = path.X(size-3);
    y3 = path.Y(size-3);

    theta1 = atan2(y1 -y2,x1 -x2);
    theta2 = atan2(y2 -y3,x2 -x3);

    theta  = angles::normalize_angle(theta1 + (theta1 - theta2));
    x0     = x1 + 1.5*std::cos(theta);
    y0     = y1 + 1.5*std::sin(theta);

    for(int i=0;i<size;i++)
    {
      path1.X(i) = path.X(i);
      path1.Y(i) = path.Y(i);
    }
    path1.X(size) = x0;
    path1.Y(size) = y0;

    return path1;
  }
}
//拟合样条曲线
void ArcLengthSpline::fitSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y)
{
    //一共分为三步，拟合曲线fit spline，路线离散化采样re-sample path，计算弧长compute arc length
    // temporary spline class only used for fitting
    Eigen::VectorXd s_approximation;
    PathData first_refined_path,second_refined_path;
    double total_arc_length;//路径总长度
    //计算对应的弧长集合,用点之间的直线长度近似替代弧长
    s_approximation = compArcLength(X,Y);
    //计算路径总长度
    total_arc_length = s_approximation(s_approximation.size()-1);
    //初始化CubicSpline对象
    CubicSpline first_spline_x,first_spline_y;
    CubicSpline second_spline_x,second_spline_y;
    // 1. 拟合样条曲线
    first_spline_x.genSpline(s_approximation,X,false);
    first_spline_y.genSpline(s_approximation,Y,false);
    // 2. 进行重采样，保证路径长度等距
    first_refined_path = resamplePath(first_spline_x,first_spline_y,total_arc_length);
    s_approximation = compArcLength(first_refined_path.X,first_refined_path.Y);
    total_arc_length = s_approximation(s_approximation.size()-1);
    // 3.二次曲线拟合及重采样
    second_spline_x.genSpline(s_approximation,first_refined_path.X,false);
    second_spline_y.genSpline(s_approximation,first_refined_path.Y,false);
    second_refined_path = resamplePath(second_spline_x,second_spline_y,total_arc_length);
    //4.将二次采样的路径传入路径中
    setRegularData(second_refined_path.X,second_refined_path.Y,second_refined_path.s);
    // 拟合样条曲线根据固定弧长
    spline_x_.genSpline(path_data_.s,path_data_.X,true);
    spline_y_.genSpline(path_data_.s,path_data_.Y,true);
    //找到停车点,
    double dis = DBL_MAX;
    double dx  = 0.0;
    double dy  = 0.0;
    for(int i=path_data_.X.size()-1;i>=0;i--)
    {
      dx = target_point_.position.x -  path_data_.X(i);
      dy = target_point_.position.y -  path_data_.Y(i);

      if((dx*dx + dy*dy) < dis)
      {
        target_point_index_ = i;
        dis = dx*dx + dy*dy;
        //std::cout<<" dis ="<< dis<<std::endl;
      }
      else
      {
        //std::cout<<" dis ="<< dis<<std::endl;
        target_point_index_ = i;
        break;
      }
    }
}
//生成2维样条曲线
void ArcLengthSpline::gen2DSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y)
{
    // 对点进行重采样处理，去除过于密集的点
    RawPath clean_path;
    clean_path = outlierRemoval(X,Y);
    // 进行样条曲线的拟合
    //为了解决最终的停车问题,在进行样条曲线拟合时,在路线的最后多加一段路,保证可以继续往前进行计算
    //1.记下路径终点坐标,防止路径延伸后,被覆盖
    target_point_.position.x = X(X.size()-1);
    target_point_.position.y = Y(Y.size()-1);
    //2.进行路径延伸
    RawPath extend_path = extendPath(clean_path);
    //3.获取样条曲线
    fitSpline(extend_path.X,extend_path.Y);
    //fitSpline(clean_path.X,clean_path.Y);
}
//根据路径弧长求取对应的点坐标
Eigen::Vector2d ArcLengthSpline::getPostion(const double s) const
{
    Eigen::Vector2d s_path;
    s_path(0) = spline_x_.getPoint(s);
    s_path(1) = spline_y_.getPoint(s);

    return s_path;
}
//获取弧长1阶导数
Eigen::Vector2d ArcLengthSpline::getDerivative(const double s) const
{
    Eigen::Vector2d ds_path;
    ds_path(0) = spline_x_.getDerivative(s);
    ds_path(1) = spline_y_.getDerivative(s);

    return ds_path;
}
//获取弧长2阶导数
Eigen::Vector2d ArcLengthSpline::getSecondDerivative(const double s) const
{
    Eigen::Vector2d dds_path;
    dds_path(0) = spline_x_.getSecondDerivative(s);
    dds_path(1) = spline_y_.getSecondDerivative(s);

    return dds_path;
}
//获取路径总弧长
double ArcLengthSpline::getLength() const
{
    return path_data_.s(path_data_.n_points-1);
}
//
double ArcLengthSpline::porjectOnSpline(const State &x) const
{
    //获取机器人当前位置以及路径上对应的位置
    Eigen::Vector2d pos;
    pos(0) = x.X;
    pos(1) = x.Y;
    double s_guess = x.s;
    Eigen::Vector2d pos_path = getPostion(s_guess);
    //初始化
    //norm表示点乘再开方，其实和sqrt(dx^2 + dy^2)意义是一样的
    double s_opt = s_guess;
    double dist = (pos-pos_path).norm();
    //param_.max_dist_proj为0.2m，超过该值
    //重新获取距离小车最近的点,将其作为刚刚到达的点
    if (dist >= param_.max_dist_proj)
    {
        //ArrayXd square()函数为原向量的每个元素都取平方，再构成新的向量
        //取平方是为了保证出现负数的情况
        //std::cout << "dist too large:" <<dist<< std::endl;
        Eigen::ArrayXd diff_x_all = path_data_.X.array() - pos(0);
        Eigen::ArrayXd diff_y_all = path_data_.Y.array() - pos(1);
        Eigen::ArrayXd dist_square = diff_x_all.square() + diff_y_all.square();
        //dist_square.data()应该是容器的首地址， + dist_square.size()是存入容器的元素个数
        std::vector<double> dist_square_vec(dist_square.data(),dist_square.data() + dist_square.size());
        //std::min_element（）返回最小值对应的指针
        auto min_iter = std::min_element(dist_square_vec.begin(),dist_square_vec.end());
        //std::cout<<"min_iter:"<<std::distance(dist_square_vec.begin(), min_iter)<<std::endl;
        //std::distance（）返回迭代器范围的元素个数，也就是找到机器人已经走过的总长度
    s_opt = path_data_.s(std::distance(dist_square_vec.begin(), min_iter));
    }
    double s_old = s_opt;
    //通过迭代求s
    for(int i=0; i<20; i++)
    {
        //获取弧长对应的点坐标
        pos_path = getPostion(s_opt);
        //求弧长对应的一阶和二阶偏导数
        Eigen::Vector2d ds_path = getDerivative(s_opt);
        Eigen::Vector2d dds_path = getSecondDerivative(s_opt);
        //误差
        Eigen::Vector2d diff = pos_path - pos;
        //雅可比和海森
        double jac = 2.0 * diff(0) * ds_path(0) + 2.0 * diff(1) * ds_path(1);
        double hessian = 2.0 * ds_path(0) * ds_path(0) + 2.0 * diff(0) * dds_path(0) +
                         2.0 * ds_path(1) * ds_path(1) + 2.0 * diff(1) * dds_path(1);
        // Newton method,-jac/hessian为迭代方向??????????
        if(hessian == 0)
        {
          std::cout<<"第"<<i<<"次迭代 "<<"s_old = "<<s_old<<"s_opt = "<<s_opt<<std::endl;
          std::cout<<"ds_path(0):"<<ds_path(0)<<std::endl;
          std::cout<<"ds_path(1):"<<ds_path(1)<<std::endl;
          std::cout<<"dds_path(0):"<<dds_path(0)<<std::endl;
          std::cout<<"dds_path(1):"<<dds_path(1)<<std::endl;
          std::cout<<"diff(0):"<<diff(0)<<std::endl;
          std::cout<<"diff(1):"<<diff(1)<<std::endl;
          return s_opt;
        }
        else
        {
          s_opt -= jac/hessian;
          s_opt = unwrapInput(s_opt);
        }
        if(std::abs(s_old - s_opt) <= 1e-5)
        {
          return s_opt;
        }
        s_old = s_opt;
    }
    // 如果在最大迭代次数后还没有达到偏差阈值，直接返回s_guess
    return s_guess;
}

bool ArcLengthSpline::IsGoReached(const State &x)
{
  Pose<double> robot_pose;
  robot_pose.position.x    = x.X;
  robot_pose.position.y    = x.Y;
  robot_pose.heading_angle = x.phi;

  int nearest_index = FindNearestPoint(robot_pose);
  //std::cout<<"nearest_index:"<<nearest_index<<"target_point_index_:"<<target_point_index_<<"path size:"<<path_data_.n_points<<std::endl;
  if(nearest_index >= target_point_index_)
  {
    if(IsPassedPoint(target_point_index_,robot_pose))
    {
      std::cout<<"超出目标点"<<std::endl;
      target_point_index_ = 65535;
      target_point_.heading_angle = DBL_MAX;
      target_point_.position.x    = DBL_MAX;
      target_point_.position.y    = DBL_MAX;
      return true;
    }
  }
  return false;
}

int ArcLengthSpline::FindNearestPoint(const Pose<double> &robot_pose)
{
  //1.先找到路径上距离机器人位置最近的点
  uint32_t target_index = 0;
  double min_distance = DBL_MAX;
  //这种情况下找到的是最近点的下一个点,如果找到的是最后一个点,则有两种情况
  for(int i = 0; i < path_data_.n_points; ++i)
  {
    double dx = path_data_.X(i) - robot_pose.position.x;
    double dy = path_data_.Y(i) - robot_pose.position.y;

    double distance = pow(dx, 2) + pow(dy, 2);
    if(distance < min_distance)
    {
      min_distance = distance;
      target_index = i;
    }
  }
  return target_index;
}

bool ArcLengthSpline::IsPassedPoint(const int &index,const Pose<double> &robot_pose)
{
  //判断方法1:其余两边的平方和小于第三条边的平方,则第三条边对应的角度为钝角
  //判断方法2:两个向量的点乘,大于0为锐角
  if(index > 0)
  {
    double vector_s_x = path_data_.X(index) - path_data_.X(index-1);
    double vector_s_y = path_data_.Y(index) - path_data_.Y(index-1);

    double cur_x      = robot_pose.position.x - path_data_.X(index);
    double cur_y      = robot_pose.position.y - path_data_.Y(index);
    if(vector_s_x * cur_x + vector_s_y * cur_y >= 0)
    {
      target_point_.position.x    = DBL_MAX;
      target_point_.position.y    = DBL_MAX;
      target_point_.heading_angle = DBL_MAX;
      target_point_index_         = 65535;
      return true;
    }
  }
  return false;
}
}
