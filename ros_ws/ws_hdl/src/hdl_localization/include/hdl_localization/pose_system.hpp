#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP

#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
 */
class PoseSystem {
public:
  typedef float T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;
public:
  PoseSystem() {
    dt = 0.01;
    first_imuq=true;
  }

  // system equation
  VectorXt f(const VectorXt& state, const VectorXt& control) {
    VectorXt next_state(16);

    Vector3t pt = state.middleRows(0, 3);
    Vector3t vt = state.middleRows(3, 3);
    Quaterniont qt(state[6], state[7], state[8], state[9]); //w,x,y,z
    qt.normalize();

    //Vector3t acc_bias = state.middleRows(10, 3);
    //Vector3t gyro_bias = state.middleRows(13, 3);

    Vector3t acc_bias(7.6163330930643187e-04,5.4245355229193000e-04,2.2816959507040164e-04);
    Vector3t gyro_bias(1.4096187361200675e-05,1.7927886559824597e-05,7.2438044807968693e-06);

    Vector3t raw_acc = control.middleRows(0, 3);
    Vector3t raw_gyro = control.middleRows(3, 3);
    VectorXt raw_quat = control.middleRows(6, 4); //w,x,y,z

    // position
    next_state.middleRows(0, 3) = pt + vt * dt;					//

    // velocity
    Vector3t g(0.0f, 0.0f, -9.80665f); 
    Vector3t acc_ = raw_acc - acc_bias;
    Vector3t acc = qt * acc_;
   
    next_state.middleRows(3, 3) = vt; // + (acc - g) * dt   acceleration didn't contribute to accuracy due to large noise

    // orientation
    Vector3t gyro = raw_gyro - gyro_bias;
   // Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2);
    Quaterniont dq(1,0,0, gyro[2] * dt / 2);
    //imu_quateration
    Quaterniont imu_q(raw_quat[0],0,0,raw_quat[3]);
 
      if(first_imuq)
    {
      first_imuq = false;
      last_imuq= imu_q;
    } 
    Quaterniont imu_dq = last_imuq.inverse()*imu_q;
    imu_dq.normalize();
     //std::cout<<"imu_q:"<< imu_q <<std::endl;
    dq.normalize();
    std::cout<< "dq:" << dq.w()<<"//" << dq.x()<<"//" << dq.y() <<"//" << dq.z() <<std::endl;
    std::cout<<"imu_dq:"<< imu_dq.w()<<"//" << imu_dq.x()<<"//" << imu_dq.y() <<"//" <<imu_dq.z() << std::endl;
  //  imu_dq.normalize();
    Quaterniont qt_ = (qt * dq).normalized();
    //Quaterniont qt_ = imu_q.normalized();
    std::cout<< "qt_:" << qt_.w()<<"//" << qt_.x()<<"//" << qt_.y() <<"//" << qt_.z() <<std::endl;
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();

    next_state.middleRows(10, 3) = state.middleRows(10, 3);		// constant bias on acceleration
    next_state.middleRows(13, 3) = state.middleRows(13, 3);		// constant bias on angular velocity

    last_imuq= imu_q;
    return next_state;
  }

  // observation equation
  VectorXt h(const VectorXt& state) const {
    VectorXt observation(7);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();

    return observation;
  }

  double dt;
  private:
  Quaterniont last_imuq;
  bool first_imuq;
};

}

#endif // POSE_SYSTEM_HPP
