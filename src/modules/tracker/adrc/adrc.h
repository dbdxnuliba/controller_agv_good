#pragma once
#include <iostream>
#include <string>
#include <math.h>
#include "mpc/deal_with_track.h"
#include "frenet_connect_cartesian.h"
#include <memory>
#include <angle.h>
#include "controller_to_ros.h"
#include "adrc_firstorder.h"
#include "adrc_secondorder.h"
namespace adrc
{
class adrc_controller
{
public:
  adrc_controller();
  ~adrc_controller();
  void SetTrack(const std::vector<Pose<float>> &path, const float &current_vel);
  void ClearTrack();
  VectorX2<float> RunAdrc(const State &x,const float &delta);
private:
  mpc::Spline2D track_;
  adrc_secondorderHdr adrc_hdr_;
};
}
