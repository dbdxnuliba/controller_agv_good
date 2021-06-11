
#include "gradient_descent_smoother/path_smoother.h"
#include "common/time_keeper.h"
//#include "math_utils.h"
namespace bz_robot
{
inline static float curvature(Vec2d x1, Vec2d x2, Vec2d x3)
{
    float result = 0;
    if(fabs(atan2(x2.y() - x1.y(), x2.x() - x1.x()) - atan2(x3.y() - x2.y(), x3.x() - x2.x())) < 0.01)
    {
        result = 0;
        //skip;
    }
    else
    {
        float dis1 = (x1 - x2).Length();
        float dis2 = (x1 - x3).Length();
        float dis3 = (x2 - x3).Length();
        float dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
        float cos_angle = dis / (2 * dis1 * dis3);
        float sin_angle = sqrt(1 - cos_angle * cos_angle);
        float r = 0.5 * dis2 / sin_angle;
        result = 1.0f / r;
    }
    return result;
}


PathSmoother::PathSmoother()
{
    RECORD_TIME();
    m_alpha = 0.1;
//    m_w_obstacle = 0.2;
//    m_w_voronoi = 0.2;
//    m_w_curvature = 0.2;
//    m_w_smoothness = 0.2;
//    m_min_road_width = 2;

    m_w_obstacle = 0.1;
    m_w_voronoi = 0.3;
    m_w_curvature = 0.5;
    m_w_smoothness = 0.5;

    m_min_turn_radius = 2;
    m_min_road_width = 1;
    m_max_iterations = 500;
}

void PathSmoother::set_smooth_times(uint32_t smooth_times)
{
    m_max_iterations = smooth_times;
}

void PathSmoother::update_map(std::shared_ptr<MapBase> p_map)
{
    mp_map = p_map;
    /// maximum possible curvature of the non-holonomic vehicle
    m_kappa_max = 1.f / (m_min_turn_radius * 1.1) * p_map->resolution();
    /// maximum distance to obstacles that is penalized
    m_obs_d_max = 1 * m_min_road_width / p_map->resolution();
    /// maximum distance for obstacles to influence the voronoi field
    m_vor_obs_d_max = 1 * m_min_road_width / p_map->resolution();
    //obsDMax的作用更显著，应该 vorObsDMax >= obsDMax。首先要不碰撞障碍物，在不碰的前提下，调整离障碍物的距离

//    m_map_width = p_map->size_in_meters_x();
//    m_map_height = p_map->size_in_meters_y();

    m_map_width = p_map->size_in_cells_x();
    m_map_height = p_map->size_in_cells_y();

    m_voronoi.buildVoronoiFromImage(p_map);
}

void PathSmoother::smoothPath(const PathData &path)
{
    //RECORD_TIME();
    int iterations = 0;
    //m_smoothed_path = path;
    m_smoothed_path.clear();
    m_smoothed_path.emplace_back(path[0]);
    for(int i = 1; i < path.size(); ++i)
    {
        if(round(path[i].position.x * 100) != round(m_smoothed_path.back().position.x * 100) ||
           round(path[i].position.y * 100) != round(m_smoothed_path.back().position.y * 100) ||
           round(path[i].heading_angle * 100) != round(m_smoothed_path.back().heading_angle * 100))
        {
            m_smoothed_path.emplace_back(path[i]);
        }
    }

    float totalWeight = m_w_smoothness + m_w_curvature + m_w_voronoi + m_w_obstacle;
    float last_avg_curvature = FLT_MAX;
    float last_curvature = 0;
    bool is_need_smooth_curvature = true;
    //Todo:make sure the cycle end condition
    const size_t path_size = m_smoothed_path.size();
    while (iterations < m_max_iterations)
    {
        //RECORD_TIME("single smooth");
        float total_move_siatance = 0;
#if 1
        for (int i = 2; i < (int)(m_smoothed_path.size() - 2); ++i)
        {
            Vec2d xim2(m_smoothed_path[i - 2].position.x, m_smoothed_path[i - 2].position.y);
            Vec2d xim1(m_smoothed_path[i - 1].position.x, m_smoothed_path[i - 1].position.y);
            Vec2d xi(m_smoothed_path[i].position.x, m_smoothed_path[i].position.y);
            Vec2d xip1(m_smoothed_path[i + 1].position.x, m_smoothed_path[i + 1].position.y);
            Vec2d xip2(m_smoothed_path[i + 2].position.x, m_smoothed_path[i + 2].position.y);

            Vec2d d_x1 = xi - xim1;
            Vec2d d_x2 = xip1 - xi;
            float Dphi = 0;
            //if(is_need_smooth_curvature)
            {
                Dphi = std::acos(d_x1.InnerProd(d_x2) / (d_x1.Length() * d_x2.Length()));
                last_curvature += Dphi;
            }

            Vec2d correction;
            if(xi.x() >= m_map_width || xi.y() >= m_map_height)
            {
                continue;
            }
            correction = correction - obstacleTerm(xi);
            if (!isOnGrid(xi + correction)) { continue; }
            total_move_siatance += correction.Length();

            correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
            //correction = correction - smoothnessTerm(xim1, xi, xip1);
            if (!isOnGrid(xi + correction)) { continue; }
            total_move_siatance += correction.Length();

            correction = correction - curvatureTerm(xim1, xi, xip1, Dphi, i, path_size);
            if (!isOnGrid(xi + correction)) { continue; }
            total_move_siatance += correction.Length();

//            correction = correction - voronoiTerm(xi);
//            if (!isOnGrid(xi + correction)) { continue; }

            //Vec2d delta_change = m_alpha * correction/totalWeight;
            //std::cout <<"iterations=" << iterations <<", point index=" << i << ", delta_change=(" << delta_change.x() << ", " << delta_change.y() << ")" << std::endl;

            xi = xi + m_alpha * correction/totalWeight;
            m_smoothed_path[i].position.x = (xi.x());
            m_smoothed_path[i].position.y = (xi.y());

            //m_smoothed_path[i - 1].heading_angle = (std::atan2(d_x1.y(), d_x1.x()));
            //PRINT_DEBUG("{:d}, {:.3f}, {:.3f}", i, m_smoothed_path[i].position.x, m_smoothed_path[i].position.y);
        }
#else
        for (int i = 1; i < path_size - 1; ++i)
        {
            Vec2d xim1(m_smoothed_path[i - 1].position.x, m_smoothed_path[i - 1].position.y);
            Vec2d xi(m_smoothed_path[i].position.x, m_smoothed_path[i].position.y);
            Vec2d xip1(m_smoothed_path[i + 1].position.x, m_smoothed_path[i + 1].position.y);

            Vec2d d_x1 = xi - xim1;
            Vec2d d_x2 = xip1 - xi;


            float Dphi = 0;
            //if(is_need_smooth_curvature)
            {
                Dphi = std::acos(d_x1.InnerProd(d_x2) / (d_x1.Length() * d_x2.Length()));
                last_curvature += Dphi;
            }


            Vec2d correction;
            if(xi.x() >= m_map_width || xi.y() >= m_map_height)
            {
                continue;
            }

            correction = correction - smoothnessTerm(xim1, xi, xip1, i, path_size);
            //printf("[%d][%d] smooth:(%.3f, %.3f)\n", iterations, i, correction.x(), correction.y());
            if (!is_valid(xim1, xi + correction, xip1)) { continue;}
            total_move_siatance += correction.Length();

            correction = correction - obstacleTerm(xi);
            //printf("[%d][%d] obstacles:(%.3f, %.3f)\n", iterations, i, correction.x(), correction.y());
            if (!is_valid(xim1, xi + correction, xip1)) { continue; }
            total_move_siatance += correction.Length();

            if(is_need_smooth_curvature)
            {
                correction = correction - curvatureTerm(xim1, xi, xip1, Dphi, i, path_size);
                //printf("[%d][%d] curvature:(%.3f, %.3f)\n", iterations, i, correction.x(), correction.y());
                if (!is_valid(xim1, xi + correction, xip1)) { continue;}
            }


//            correction = correction - voronoiTerm(xi);
//            //printf("[%d][%d] voronoi:(%.3f, %.3f)\n", iterations, i, correction.x(), correction.y());
//            if (!is_valid(xim1, xi + correction, xip1)) { continue;}

            //Vec2d delta_change = m_alpha * correction/totalWeight;
            //std::cout <<"iterations=" << iterations <<", point index=" << i << ", delta_change=(" << delta_change.x() << ", " << delta_change.y() << ")" << std::endl;

            xi = xi + m_alpha * correction/totalWeight;
            m_smoothed_path[i].position.x = (xi.x());
            m_smoothed_path[i].position.y = (xi.y());
            Vec2d Dxi = xi - xim1;
            m_smoothed_path[i - 1].heading_angle = (std::atan2(Dxi.y(), Dxi.x()));
            //PRINT_DEBUG("{:d}, {:.3f}, {:.3f}", i, m_smoothed_path[i].position.x, m_smoothed_path[i].position.y);
        }
#endif
        ++iterations;
        if(last_curvature < last_avg_curvature)
        {
            last_avg_curvature = last_curvature;
        }
        else
        {
            is_need_smooth_curvature = true;
        }
        //PRINT_DEBUG("[{:d}]total move distance = {:.3f}", iterations, total_move_siatance);
//        if(total_move_siatance < 0.1)
//        {
//            break;
//        }

    }
    for (int i = 1; i < (int)(m_smoothed_path.size() - 1); ++i)
    {
        Vec2d xim1(m_smoothed_path[i - 1].position.x, m_smoothed_path[i - 1].position.y);
        Vec2d xi(m_smoothed_path[i].position.x, m_smoothed_path[i].position.y);
        Vec2d xip1(m_smoothed_path[i + 1].position.x, m_smoothed_path[i + 1].position.y);
        Vec2d d_x1 = xi - xim1;
        Vec2d d_x2 = xip1 - xi;
        const FLOAT_T temp_dt0 = d_x1.Length();
        const FLOAT_T temp_dt1 = d_x2.Length();
        const FLOAT_T temp_dx0 = d_x1.x();
        const FLOAT_T temp_dy0 = d_x1.y();
        const FLOAT_T temp_dx1 = d_x2.x();
        const FLOAT_T temp_dy1 = d_x2.y();
        const FLOAT_T temp_sum_1 = 1.0f / temp_dt0 + temp_dt1;
        const FLOAT_T temp_dx = temp_dt0 * temp_sum_1 + temp_dx0 + temp_dt1 * temp_sum_1 * temp_dx1;
        const FLOAT_T temp_dy = temp_dt0 * temp_sum_1 + temp_dy0 + temp_dt1 * temp_sum_1 * temp_dy1;
        m_smoothed_path[i].heading_angle = atan2(temp_dy, temp_dx);
    }
    m_smoothed_path[0] = path[0];
    m_smoothed_path.back() = path.back();
}

Vec2d PathSmoother::obstacleTerm(Vec2d xi)
{
    Vec2d gradient;
    // the distance to the closest obstacle from the current node
    float obsDst = m_voronoi.getDistance(xi.x(), xi.y());
    //PRINT_DEBUG("%f, %f", xi.x(), xi.y());
    // the vector determining where the obstacle is
    int x = (int)xi.x();
    int y = (int)xi.y();
    // if the node is within the map
    if (x < m_map_width && x >= 0 && y < m_map_height && y >= 0)
    {
        Vec2d obsVct(xi.x() - m_voronoi.GetClosetObstacleCoor(xi).x(),
        xi.y() - m_voronoi.GetClosetObstacleCoor(xi).y());
        //obsDst should be equal to the length of obsVct. However, their difference may be larger than 1m.
        //    std::cout << "(==) dis to closest obs = " << obsDst << ", Vector Mod = " << obsVct.length() << std::endl;
        // the closest obstacle is closer than desired correct the path for that
        // obsDMax = 2m
        if(obsDst == 0)
        {
            obsDst = 1e5;
        }
        if (obsDst < m_obs_d_max && obsDst > 1e-6)
        //if (obsDst > m_obs_d_max && obsDst > 1e-6)
        {
            gradient = m_w_obstacle * 2 * (obsDst - m_obs_d_max) * obsVct / obsDst;
            return gradient;
        }
    }
    return gradient;
}

Vec2d PathSmoother::voronoiTerm(Vec2d xi) {
  Vec2d gradient;
  float obsDst = m_voronoi.getDistance(xi.x(), xi.y());
  Vec2d obsVct(xi.x() - m_voronoi.GetClosetObstacleCoor(xi).x(),
               xi.y() - m_voronoi.GetClosetObstacleCoor(xi).y());

  float edgDst = 0.0;
  Vec2i closest_edge_pt = m_voronoi.GetClosestVoronoiEdgePoint(xi, edgDst);
  Vec2d edgVct(xi.x() - closest_edge_pt.x(), xi.y() - closest_edge_pt.y());

  if (obsDst < m_vor_obs_d_max && obsDst > 1e-6)
  //if (obsDst > m_vor_obs_d_max)
  {
    if (edgDst > 0) {
      Vec2d PobsDst_Pxi = obsVct / obsDst;
      Vec2d PedgDst_Pxi = edgVct / edgDst;
//      float PvorPtn_PedgDst = alpha * obsDst * std::pow(obsDst - vorObsDMax, 2) /
//                              (std::pow(vorObsDMax, 2) * (obsDst + alpha) * std::pow(edgDst + obsDst, 2));
      float PvorPtn_PedgDst = (m_alpha / m_alpha + obsDst) *
                              (pow(obsDst - m_vor_obs_d_max, 2) / pow(m_vor_obs_d_max, 2)) * (obsDst / pow(obsDst + edgDst, 2));

//      float PvorPtn_PobsDst = (alpha * edgDst * (obsDst - vorObsDMax) * ((edgDst + 2 * vorObsDMax + alpha)
//                                                                         * obsDst + (vorObsDMax + 2 * alpha) * edgDst + alpha * vorObsDMax))
//                              / (std::pow(vorObsDMax, 2) * std::pow(obsDst + alpha, 2) * std::pow(obsDst + edgDst, 2));
      float PvorPtn_PobsDst = (m_alpha / (m_alpha + obsDst)) *
                              (edgDst / (edgDst + obsDst)) * ((obsDst - m_vor_obs_d_max) / pow(m_vor_obs_d_max, 2))
                              * (-(obsDst - m_vor_obs_d_max) / (m_alpha + obsDst) - (obsDst - m_vor_obs_d_max) / (obsDst + edgDst) + 2);
      gradient = m_w_voronoi * (PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi) * 100;
      return gradient;
    }
    return gradient;
  }
  return gradient;
}


Vec2d PathSmoother::curvatureTerm(Vec2d xim1, Vec2d xi, Vec2d xip1, const float &Dphi,
                                  const size_t& index, const size_t& path_size)
{
    Vec2d gradient;
    // the vectors between the nodes
    Vec2d Dxi = xi - xim1;
    //Vec2d Dxi = xim1 - xi;
    Vec2d Dxip1 = xip1 - xi;
    // orthogonal complements vector
    Vec2d p1, p2;

    float absDxi = Dxi.Length();
    float absDxip1 = Dxip1.Length();

    // ensure that the absolute values are not null
    if (absDxi > 0 && absDxip1 > 0)
    {
        //float Dphi = std::acos(Clamp<float>(Dxi.InnerProd(Dxip1) / (absDxi * absDxip1), -1, 1));
        float kappa = Dphi / absDxi;
        //printf("kappa = %.3f, curvature = %.3f\n", kappa, curvature(xim1, xi, xip1));
        if (kappa <= m_kappa_max)
        {
            Vec2d zeros;
            //      std::cout << "curvatureTerm is 0 because kappa(" << kappa << ") < kappamax(" << kappaMax << ")" << std::endl;
            return zeros;
        }
        else
        {
            //代入原文公式(2)与(3)之间的公式. 参考：
            // Dolgov D, Thrun S, Montemerlo M, et al. Practical search techniques in path planning for
            // autonomous driving[J]. Ann Arbor, 2008, 1001(48105): 18-80.
            float absDxi1Inv = 1 / absDxi;
            float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
            //float u = -absDxi1Inv * PDphi_PcosDphi;
            float u = absDxi1Inv * PDphi_PcosDphi;
            p1 = xi.ort(-xip1) / (absDxi * absDxip1);//公式(4)
            p2 = -xip1.ort(xi) / (absDxi * absDxip1);
            float s = Dphi / (absDxi * absDxi);
            Vec2d ones(1, 1);
            Vec2d ki = u * (-p1 - p2) - (s * ones);
            Vec2d kim1 = u * p2 + (s * ones);
            Vec2d kip1 = u * p1;
            float w_ei = 1;
            if(index < (path_size / 2))
            {
                w_ei = 1.0f / (1 + exp(-index));
            }
            else
            {
                w_ei = 1.0f / (1 + exp(-(path_size - 1 -index)));
            }
            gradient = w_ei * m_w_curvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);

            if (std::isnan(gradient.x()) || std::isnan(gradient.y()))
            {
                //        std::cout << "nan values in curvature term" << std::endl;
                Vec2d zeros;
                //        std::cout << "curvatureTerm is 0 because gradient is non" << std::endl;
                return zeros;
            }
            else
            {
                //        std::cout << "curvatureTerm is (" << gradient.x() << ", " << gradient.y() << ")" << std::endl;
                return gradient;
            }
        }
    }
    else
    {
        PRINT_DEBUG("abs values not larger than 0, absDxi{}, absDxi{}", absDxi, absDxip1);
        Vec2d zeros;
        PRINT_DEBUG("curvatureTerm is 0 because abs values not larger than 0");
        return zeros;
    }
}

Vec2d PathSmoother::smoothnessTerm(Vec2d xim2, Vec2d xim1, Vec2d xi, Vec2d xip1, Vec2d xip2)
{
  // according to paper "Practical search techniques in path planning for autonomous driving"
  return m_w_smoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}

Vec2d PathSmoother::smoothnessTerm(Vec2d xim, Vec2d xi, Vec2d xip, const size_t& index, const size_t& path_size)
{
    // according to paper "Practical search techniques in path planning for autonomous driving"
    float w_ei = 1;
    if(index < (path_size / 4))
    {
        w_ei = 1.0f / (1 + exp(-index));
    }
    else if(index > (path_size *0.75))
    {
        w_ei = 1.0f / (1 + exp(-(path_size - 1 -index)));
    }
    else
    {
        w_ei = 1.0f;
    }
    return w_ei * m_w_smoothness * (-4) * (xip - 2*xi + xim);
}

bool PathSmoother::isOnGrid(Vec2d vec)
{
  if (vec.x() >= 0 && vec.x() < m_map_width &&
      vec.y() >= 0 && vec.y() < m_map_height &&
        mp_map->cost(vec.x(), vec.y()) < mp_map->obstacles_cost() - 1)
  {
    return true;
  }
  return false;
}

inline bool PathSmoother::is_valid(const Vec2d &xim, const Vec2d &xi, const Vec2d &xip)
{
    if(xi.x() < 0 || xi.x() >= m_map_width)
    {
        return false;
    }
    if(xi.y() < 0 || xi.x() >= m_map_height)
    {
        return false;
    }
    if(mp_map->cost(xi.x(), xi.y()) > mp_map->obstacles_cost() - 2)
    {
        return false;
    }
//    if(curvature(xim, xi, xip) > m_kappa_max)
//    {
//        return false;
//    }
    return true;
}
}
