#include <iostream>
#include <math.h>
#include <float.h>
#include <string>
#include <vector>
#include "frenet_cartesian.h"
#include "common/common.h"
#include "common/geometry.h"


using namespace std;
namespace bz_robot
{
namespace frenet_polynomial_planner
{

static VectorX2<float> closest_point_on_line(VectorX2<float> cur_pose, VectorX2<float> a, VectorX2<float> b)
{
    VectorX2<float> point;
    VectorX2<float> a_p = cur_pose - a;
    VectorX2<float> a_b = b - a;
    float len_square = a_b.x * a_b.x + a_b.y * a_b.y;
    float ab_ap_product = a_b.x * a_p.x + a_b.y * a_p.y;
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

static float calculate_cte(VectorX2<float> cur_point,
                     VectorX2<float> point_from,
                     VectorX2<float> point_to)
{
    VectorX2<float> cloest_point = closest_point_on_line(cur_point, point_from, point_to);
    const float dx = cur_point.x - cloest_point.x;
    const float dy = cur_point.y - cloest_point.y;
    float cte = hypot(dx, dy);
    //determine if CTE is negative or positive so we can steer in the direction we need
    //Is the car to the right or to the left of the upcoming waypoint
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


// Calculate distance between two points
inline float distance(float x1, float y1, float x2, float y2)
{
    return hypot(x2 - x1, y2 - y1);
}

// Calculate closest waypoint to current x, y position
static int closest_waypoint(float x, float y, const vector<float> &maps_x,
                            const vector<float> &maps_y)
{
    float closestLen = DBL_MAX; //large number
    int closest_waypoint = 0;

    for(int i = 0; i < maps_x.size(); ++i)
    {
        float map_x = maps_x[i];
        float map_y = maps_y[i];
        float dist = distance(x, y, map_x, map_y);

        if(dist < closestLen)
        {
            closestLen = dist;
            closest_waypoint = i;
        }
    }

    return closest_waypoint;
}

// Returns next waypoint of the closest waypoint
int next_waypoint(float x, float y, float theta, const vector<float> &maps_x,
                  const vector<float> &maps_y)
{
    int closest_waypoint_index = closest_waypoint(x, y, maps_x, maps_y);

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

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<float> get_frenet(float x, float y, float theta,
                          vector<float> maps_x,
                          vector<float> maps_y)
{
    float s_sign = 1.0;
    if(maps_x.size() != maps_y.size())
    {
        return {0,0};
    }
    if(maps_x.size() < 2)
    {
        return {0,0};
    }
    int next_wp = next_waypoint(x, y, theta, maps_x, maps_y);
    bool is_insert_front_point = false;
    if(next_wp == 0)
    {
        is_insert_front_point = true;
        s_sign = -1.0;
        float len = hypot(x - maps_x.front(), y - maps_y.front());
        if(round(len * 1000) == 0)
        {
            len = 1.0;
        }
        float theta = atan2(maps_y[1] - maps_y[0], maps_x[1] - maps_x[0]);
        maps_x.insert(maps_x.begin(), maps_x[0] - len * cos(theta));
        maps_y.insert(maps_y.begin(), maps_y[0] - len * sin(theta));
        //printf("insert point({:.3f}, {:.3f})\n", maps_x.front(), maps_y.front());
    }
    else if (next_wp == maps_x.size())
    {
        float len = hypot(x - maps_x.back(), y - maps_y.back());
        if(round(len * 1000) == 0)
        {
            len = 1.0;
        }
        float theta = atan2(maps_y[next_wp-1] - maps_y[next_wp-2], maps_x[next_wp-1] - maps_x[next_wp-2]);
        maps_x.insert(maps_x.end(), maps_x.back() + len * cos(theta));
        maps_y.insert(maps_y.end(), maps_y.back() + len * sin(theta));
        //printf("len = {:.3f}, theta = {:.3f}, dx = {:.3f}, dy = {:.3f}\n", len, theta, len * cos(theta), len * sin(theta));
    }
    next_wp = next_waypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;


    VectorX2<float> prev_point(maps_x[prev_wp], maps_y[prev_wp]);
    VectorX2<float> next_point(maps_x[next_wp], maps_y[next_wp]);
    VectorX2<float> cur_point(x, y);
    float frenet_d = calculate_cte(cur_point, prev_point, next_point);
//    printf("cur point: ({:.3f}, {:.3f}), prev point: ({:.3f}, {:.3f}), next point: ({:.3f}, {:.3f})\n",
//           cur_point.x, cur_point.y, prev_point.x, prev_point.y, next_point.x, next_point.y);


    float frenet_s = 0;
    if(is_insert_front_point)
    {
        float len = hypot(x - maps_x[1], y - maps_y[1]);
        //float heading = atan2(maps_y[0] - maps_y[1], maps_x[0] - maps_x[1]);
        float delta_s = sqrt(len * len - frenet_d *frenet_d);
        if(isnan(delta_s))
        {
            delta_s = 0;
        }
        frenet_s = -delta_s;
    }
    else
    {
        float len = hypot(x - prev_point.x, y - prev_point.y);
        float delta_s = sqrt(len * len - frenet_d *frenet_d);
        if(isnan(delta_s))
        {
            delta_s = 0;
        }
        //printf("d = {:.3f}, delta s = {:.3f}\n", frenet_d, delta_s);
        float heading = atan2(next_point.y - prev_point.y, next_point.x - prev_point.x);
        float delta_heading = atan2(frenet_d,delta_s);
        float c_heading = atan2(cur_point.y - prev_point.y, cur_point.x - prev_point.x);
        float dx = cur_point.x - prev_point.x;
        float dy = cur_point.y - prev_point.y;
//        printf("vector:heading:{:.3f} + sd {:.3f} = {:.3f}, seg_s = {:.3f}, dx = {:.3f}, dy = {:.3f}\n",
//               heading, delta_heading, c_heading, len, dx, dy);

        for(int i = 0; i < prev_wp; ++i)
        {
            frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
        }
        frenet_s += delta_s;
    }
    return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<float> get_xy(float s, float d, vector<float> maps_s,
                      vector<float> maps_x,
                      vector<float> maps_y)
{
    int prev_wp = -1;
    if(s <= maps_s[0])
    {
        float len = (maps_s[0] - s)+1;
        float theta = atan2(maps_y[1] - maps_y[0], maps_x[1] - maps_x[0]);
        maps_x.insert(maps_x.begin(), maps_x[0] - (len) * cos(theta));
        maps_y.insert(maps_y.begin(), maps_y[0] - (len) * sin(theta));
        maps_s.insert(maps_s.begin(), s-1);
        //printf("s = {:.3f}, s[0] = {:.3f}\n", s, maps_s[0]);
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

//    printf("prev point[{:d}]: ({:.3f}, {:.3f}), next point[{:d}]: ({:.3f}, {:.3f})\n",
//           prev_wp, maps_x[prev_wp], maps_y[prev_wp], wp2, maps_x[wp2], maps_y[wp2]);
    float heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                           (maps_x[wp2] - maps_x[prev_wp]));
    float delta_s = s - maps_s[prev_wp];
    float delta_heading = atan2(d, delta_s);
    //printf("d = {:.3f}, delta s = {:.3f}\n", d, delta_s);
    float c_heading = heading + delta_heading;
    float seg_s = hypot(delta_s, d);
    const float dx = seg_s * cos(c_heading);
    const float dy = seg_s * sin(c_heading);
//    printf("vector:heading:{:.3f} + sd {:.3f} = {:.3f}, seg_s = {:.3f}, dx = {:.3f}, dy = {:.3f}\n",
//           heading, delta_heading, c_heading, seg_s, dx, dy);
    float x = maps_x[prev_wp] + dx;
    float y = maps_y[prev_wp] + dy;
    //    // the x,y,s along the segment
    //    float seg_s = (s - maps_s[prev_wp]);

    //    float seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    //    float seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    //    float perp_heading = heading - M_PI_2;

    //    float x = seg_x + d * cos(perp_heading);
    //    float y = seg_y + d * sin(perp_heading);

    return {x, y, c_heading};
}
}
}
