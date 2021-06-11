#include <vector>
#include "common/common.h"
#include "common/geometry.h"
#include <float.h>
namespace bz_robot {
class Frenet;
using FrenetHdr = std::shared_ptr<Frenet>;
using FrenetPtr = Frenet*;
using FrenetCPtr = const Frenet*;
class Frenet
{
public:
  Frenet();
  ~Frenet();
public:
  std::vector<float> ToFrenet(float x, float y, float theta,
                             std::vector<float> maps_x,
                             std::vector<float> maps_y);

  std::vector<float> FromFrenet(float s, float d, std::vector<float> maps_s,
                         std::vector<float> maps_x,
                         std::vector<float> maps_y);
private:
  VectorX2<float> ComputeFootPrint(VectorX2<float> cur_pose, VectorX2<float> a,
                                           VectorX2<float> b);
  float ComputeDisToPath(VectorX2<float> cur_point,VectorX2<float> point_from,
                              VectorX2<float> point_to);
  inline float Distance(float x1, float y1, float x2, float y2)
  {
      return hypot(x2 - x1, y2 - y1);
  }
  int ClosestWayPoint(float x, float y, const std::vector<float> &maps_x,
                              const std::vector<float> &maps_y);
  int NextWayPoint(float x, float y, float theta, const std::vector<float> &maps_x,
                    const std::vector<float> &maps_y);
};

}
