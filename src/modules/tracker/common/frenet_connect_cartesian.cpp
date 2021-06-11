#include "common/frenet_connect_cartesian.h"
namespace bz_robot {

Frenet::Frenet()
{

}

Frenet::~Frenet()
{

}

std::vector<float> Frenet::ToFrenet(float x, float y, float theta, std::vector<float> maps_x, std::vector<float> maps_y)
{
  float s_sign = 1.0;
  //当xy大小不一致或是点数太少都返回0,0
  if(maps_x.size() != maps_y.size())
  {
    return {0,0};
  }
  if(maps_x.size() < 2)
  {
    return {0,0};
  }
  //理解为将要到达的点
  int next_wp = NextWayPoint(x, y, theta, maps_x, maps_y);
  bool is_insert_front_point = false;
  if(next_wp == 0)
  {
    is_insert_front_point = true;
    s_sign = -1.0;
    //机器人到路径起点的距离
    float len = hypot(x - maps_x.front(), y - maps_y.front());
    //std::round()是四舍五入到最近的整数，如果满足if条件说明机器人位置和起点几乎重合
    if(round(len * 1000) == 0)
    {
      len = 1.0;//发现机器人位于起点，要反向补充1m的路径
    }
    float theta = atan2(maps_y[1] - maps_y[0], maps_x[1] - maps_x[0]);
    maps_x.insert(maps_x.begin(), maps_x[0] - len * cos(theta));
    maps_y.insert(maps_y.begin(), maps_y[0] - len * sin(theta));
    //printf("insert point(%.3f, %.3f)\n", maps_x.front(), maps_y.front());
  }
  else if (next_wp == maps_x.size())
  {
    float len = hypot(x - maps_x.back(), y - maps_y.back());
    //如果满足if条件说明机器人位置和终点几乎重合
    if(round(len * 1000) == 0)
    {
      len = 1.0;//发现机器人位于起点，要正向补充1m的路径
    }
    float theta = atan2(maps_y[next_wp-1] - maps_y[next_wp-2], maps_x[next_wp-1] - maps_x[next_wp-2]);
    maps_x.insert(maps_x.end(), maps_x.back() + len * cos(theta));
    maps_y.insert(maps_y.end(), maps_y.back() + len * sin(theta));
    //printf("len = %.3f, theta = %.3f, dx = %.3f, dy = %.3f\n", len, theta, len * cos(theta), len * sin(theta));
  }
  //上面的延伸路径是为了保证prev_wp不越界
  next_wp = NextWayPoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  //求机器人到线路的垂足距离frenet_d，左为正，右为负
  VectorX2<float> prev_point(maps_x[prev_wp], maps_y[prev_wp]);
  VectorX2<float> next_point(maps_x[next_wp], maps_y[next_wp]);
  VectorX2<float> cur_point(x, y);
  float frenet_d = ComputeDisToPath(cur_point, prev_point, next_point);
  //求机器人已经走过的线路长度frenet_s
  //float frenet_s = 0;
//  if(is_insert_front_point)
//  {
//    //机器人还没有超过线路真正的起点，frenet_s为负
//    float len = hypot(x - maps_x[1], y - maps_y[1]);
//    float delta_s = sqrt(len * len - frenet_d *frenet_d);
//    frenet_s = -delta_s;
//  }
//  else
//  {
//    //机器人已经走过的线路长度
//    float len = hypot(x - prev_point.x, y - prev_point.y);
//    float delta_s = sqrt(len * len - frenet_d *frenet_d);

//    for(int i = 0; i < prev_wp; ++i)
//    {
//      frenet_s += Distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
//    }
//    frenet_s += delta_s;
//  }
  //todo因为主函数中有其他方法可以计算frenet_s，这里为减小计算量，暂时注释掉，后期进行修改
  return {next_wp, frenet_d};
}

std::vector<float> Frenet::FromFrenet(float s, float d, std::vector<float> maps_s, std::vector<float> maps_x, std::vector<float> maps_y)
{
  int prev_wp = -1;
  if(s <= maps_s[0])
  {
      float len = (maps_s[0] - s)+1;
      float theta = atan2(maps_y[1] - maps_y[0], maps_x[1] - maps_x[0]);
      maps_x.insert(maps_x.begin(), maps_x[0] - (len) * cos(theta));
      maps_y.insert(maps_y.begin(), maps_y[0] - (len) * sin(theta));
      maps_s.insert(maps_s.begin(), s-1);
      //printf("s = %.3f, s[0] = %.3f\n", s, maps_s[0]);
  }
  else if (s > maps_s[maps_s.size()-1])
  {
      int size = maps_s.size();
      float len = fabs(s - maps_s[size-1]);
      float theta = atan2(maps_y[size-1] - maps_y[size-2], maps_x[size-1] - maps_x[size-2]);
      maps_x.insert(maps_x.end(), maps_x[size-1] + len * cos(theta));
      maps_y.insert(maps_y.end(), maps_y[size-1] + len * sin(theta));
      maps_s.insert(maps_s.end(), s);
  }

  while(s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
  {
      ++prev_wp;
  }

  int wp2 = (prev_wp + 1);

//    printf("prev point[%d]: (%.3f, %.3f), next point[%d]: (%.3f, %.3f)\n",
//           prev_wp, maps_x[prev_wp], maps_y[prev_wp], wp2, maps_x[wp2], maps_y[wp2]);
  float heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
  float delta_s = s - maps_s[prev_wp];
  float delta_heading = atan2(d, delta_s);
  //printf("d = %.3f, delta s = %.3f\n", d, delta_s);
  float c_heading = heading + delta_heading;
  float seg_s = hypot(delta_s, d);
  const float dx = seg_s * cos(c_heading);
  const float dy = seg_s * sin(c_heading);

  float x = maps_x[prev_wp] + dx;
  float y = maps_y[prev_wp] + dy;

  return {x, y, c_heading};
}

VectorX2<float> Frenet::ComputeFootPrint(VectorX2<float> cur_pose, VectorX2<float> a, VectorX2<float> b)
{
  VectorX2<float> point;
  VectorX2<float> a_p = cur_pose - a;
  VectorX2<float> a_b = b - a;
  float len_square = a_b.x * a_b.x + a_b.y * a_b.y;
  float ab_ap_product = a_b.x * a_p.x + a_b.y * a_p.y;
  //distance表示的是起点与垂足点的连线向量比上起点与终点向量的比值，同向为正，异向为负
  float distance = ab_ap_product / len_square;
  if(distance < 0)
  {
      point = a;
  }
  else if(distance > 1)
  {
      point = b;
  }
  else
  {
      point = a_b;
      point.x *= distance;
      point.y *= distance;
      point = a + point;
  }
  return point;
}

float Frenet::ComputeDisToPath(VectorX2<float> cur_point, VectorX2<float> point_from, VectorX2<float> point_to)
{
  //获取机器人到线路的垂足点
  VectorX2<float> cloest_point = ComputeFootPrint(cur_point, point_from, point_to);
  const float dx = cur_point.x - cloest_point.x;
  const float dy = cur_point.y - cloest_point.y;
  //获取垂线长度
  float cte = hypot(dx, dy);
  //机器人位于线路左侧，angle_diff > 0，距离为正，否则为负
  VectorX2<float> to_cur_vec = cur_point - point_from;
  VectorX2<float> to_point_vec = point_to - point_from;
  float angle1 = atan2(to_cur_vec.y, to_cur_vec.x);
  float angle2 = atan2(to_point_vec.y, to_point_vec.x);
  float angle_diff = constraint_angle_r(angle1 - angle2, -M_PI, M_PI);
  if(angle_diff < 0)
  {
    cte *= -1.0;
  }
  return cte;
}

int Frenet::ClosestWayPoint(float x, float y, const std::vector<float> &maps_x, const std::vector<float> &maps_y)
{
  float closestLen = DBL_MAX; //large number
  int closest_waypoint = 0;

  for(int i = 0; i < maps_x.size(); ++i)
  {
      float map_x = maps_x[i];
      float map_y = maps_y[i];
      float dist = Distance(x, y, map_x, map_y);

      if(dist < closestLen)
      {
          closestLen = dist;
          closest_waypoint = i;
      }
  }

  return closest_waypoint;
}

int Frenet::NextWayPoint(float x, float y, float theta, const std::vector<float> &maps_x, const std::vector<float> &maps_y)
{
  int closest_waypoint_index = ClosestWayPoint(x, y, maps_x, maps_y);

  float map_x = maps_x[closest_waypoint_index];
  float map_y = maps_y[closest_waypoint_index];

  float heading = atan2((map_y - y), (map_x - x));

  //    float angle = fabs(theta - heading);
  //    angle = std::min(2 * M_PI - angle, angle);

  float angle = fabs(constraint_angle_r(theta - heading, -M_PI, M_PI));
  //angle = std::min(2 * M_PI - angle, angle);

#if 0
  if(angle > M_PI_2)
  {
      ++closest_waypoint_index;

      if(closest_waypoint_index == maps_x.size())
      {
          closest_waypoint_index = 0;
      }
  }
#else
  if (fabs(angle) > M_PI_2)
  {
      ++closest_waypoint_index;
  }
#endif
  return closest_waypoint_index;
}

}
