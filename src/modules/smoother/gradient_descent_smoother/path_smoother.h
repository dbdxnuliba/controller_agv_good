#ifndef PATH_SMOOTHER_PATH_SMOOTHER_H
#define PATH_SMOOTHER_PATH_SMOOTHER_H

#include <cmath>
#include <vector>
#include <memory>


#include "pose2d.h"
#include "vec2d.h"
#include "dynamic_voronoi.h"
//#include "constants.h"
#include "common/geometry.h"
#include "modules/map/map_base.h"
#include "common/time_keeper.h"
#include "common/data_types.h"

namespace bz_robot
{
class PathSmoother
{
public:
    PathSmoother();
    ~PathSmoother()
    {
        m_original_path.clear();
        m_smoothed_path.clear();
    }
    void set_smooth_times(uint32_t smooth_times);
    void update_map(std::shared_ptr<MapBase> p_map);
    void smoothPath(const PathData &path);

//    std::vector<Pose<float>> getOriginalPath() {return m_original_path;}

    PathData getSmoothedPath()
    {
        return m_smoothed_path;
    }

private:

    Vec2d obstacleTerm(Vec2d xi);//障碍物项，用于约束路径远离障碍物

    Vec2d curvatureTerm(Vec2d xi0, Vec2d xi1, Vec2d xi2, const float &Dphi,
                        const size_t &index, const size_t &path_size);//曲率项，用于保证可转弯性及通行性

    //平滑项，用于将节点等距分布并尽量保持同一个方向
    Vec2d smoothnessTerm(Vec2d xim2, Vec2d xim1, Vec2d xi, Vec2d xip1, Vec2d xip2);
    Vec2d smoothnessTerm(Vec2d xim, Vec2d xi, Vec2d xip, const size_t &index, const size_t &path_size);

    Vec2d voronoiTerm(Vec2d xi);

    bool isOnGrid(Vec2d vec);
    inline bool is_valid(const Vec2d& xim, const Vec2d& xi, const Vec2d& xip);

    friend class GradientDescentSmoother;
    float m_min_turn_radius;
    float m_min_road_width;
    uint32_t m_max_iterations;
    /// maximum possible curvature of the non-holonomic vehicle
    float m_kappa_max;
    /// maximum distance to obstacles that is penalized
    float m_obs_d_max;
    /// maximum distance for obstacles to influence the voronoi field
    float m_vor_obs_d_max;
    //obsDMax的作用更显著，应该 vorObsDMax >= obsDMax。首先要不碰撞障碍物，在不碰的前提下，调整离障碍物的距离

    /// falloff rate for the voronoi field
    float m_alpha = 0.1;
    float m_w_obstacle = 0.2;
    float m_w_voronoi = 0.2;
    float m_w_curvature = 0.2;
    float m_w_smoothness = 0.2;

    DynamicVoronoi m_voronoi;
//    cv::Mat map_img_;
    int m_map_width;
    int m_map_height;
    PathData m_original_path;
    PathData m_smoothed_path;
    std::shared_ptr<MapBase> mp_map;
};
}


#endif // PATH_SMOOTHER_PATH_SMOOTHER_H
