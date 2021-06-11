#include "cubic_spline.h"

namespace bz_robot
{
  Spline::Spline()
  {

  }
  // d_i * (x_-x_i)^3 + c_i * (x_-x_i)^2 + b_i * (x_-x_i) + a_i
  Spline::Spline(Vec_f x, Vec_f y):
    x_(x), y_(y), nx_(x.size()), h_(vec_diff(x)), a_(y)
  {
    Eigen::MatrixXf A = CalculateA();
    Eigen::VectorXf B = CalculateB();
    Eigen::VectorXf c_eigen = A.colPivHouseholderQr().solve(B);
    float * c_pointer = c_eigen.data();
    //Eigen::Map<Eigen::VectorXf>(c_, c_eigen.rows(), 1) = c_eigen;
    c_.assign(c_pointer, c_pointer+c_eigen.rows());

    for(int i=0; i<nx_-1; i++){
      d_.push_back((c_[i+1]-c_[i])/(3.0*h_[i]));
      b_.push_back((a_[i+1] - a_[i])/h_[i] - h_[i] * (c_[i+1] + 2*c_[i])/3.0);
    }
  }

  float Spline::CalculatePositionFromS(float s)
  {
    if(s<x_.front() || s>x_.back()){
      throw std::invalid_argument( "received value out of the pre-defined range" );
    }
    int seg_id = Bisect(s, 0, nx_);
    float dx = s - x_[seg_id];
    return a_[seg_id] + b_[seg_id] * dx + c_[seg_id] * dx * dx + d_[seg_id] * dx * dx * dx;
  }

  float Spline::CalculateDrivate(float s)
  {
    if(s<x_.front() || s>x_.back()){
      throw std::invalid_argument( "received value out of the pre-defined range" );
    }
    int seg_id = Bisect(s, 0, nx_-1);
    float dx = s - x_[seg_id];
    return b_[seg_id]  + 2 * c_[seg_id] * dx + 3 * d_[seg_id] * dx * dx;
  }

  float Spline::CalculateSecondDrivate(float s)
  {
    if(s<x_.front() || s>x_.back()){
      throw std::invalid_argument( "received value out of the pre-defined range" );
    }
    int seg_id = Bisect(s, 0, nx_);
    float dx = s - x_[seg_id];
    return 2 * c_[seg_id] + 6 * d_[seg_id] * dx;
  }

  Eigen::MatrixXf Spline::CalculateA()
  {
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx_, nx_);
    A(0, 0) = 1;
    for(int i=0; i<nx_-1; i++){
      if (i != nx_-2){
        A(i+1, i+1) = 2 * (h_[i] + h_[i+1]);
      }
      A(i+1, i) = h_[i];
      A(i, i+1) = h_[i];
    }
    A(0, 1) = 0.0;
    A(nx_-1, nx_-2) = 0.0;
    A(nx_-1, nx_-1) = 1.0;
    return A;
  }

  Eigen::VectorXf Spline::CalculateB()
  {
    Eigen::VectorXf B = Eigen::VectorXf::Zero(nx_);
    for(int i=0; i<nx_-2; i++){
      B(i+1) = 3.0*(a_[i+2]-a_[i+1])/h_[i+1] - 3.0*(a_[i+1]-a_[i])/h_[i];
    }
    return B;
  }

  int Spline::Bisect(float t, int start, int end)
  {
    int mid = (start+end)/2;
    if (t==x_[mid] || end-start<=1)
    {
      return mid;
    }
    else if (t>x_[mid])
    {
      return Bisect(t, mid, end);
    }
    else
    {
      return Bisect(t, start, mid);
    }
  }
}
