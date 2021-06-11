#pragma once

#include <float.h>
#include <stdint.h>
#include <iostream>
#include <ostream>
#include <sstream>
#include <vector>
#include "geometry.h"
#include "common/common.h"
#include "common/print.h"
#include "common/json.hpp"

namespace bz_robot
{
class MapBase
{
public:
    MapBase(){};
    virtual ~MapBase(){};
    virtual inline std::vector<std::vector<int8_t>> map_data()=0;
    virtual inline std::string name() {return "";}
    virtual inline void update() = 0;
//    virtual unsigned char* char_map() = 0;
    virtual inline bool set_cost(const uint32_t &mx, const uint32_t &my, const uint8_t &cost) = 0;
    //virtual bool map_to_world(const float &m_x, const float &m_y, float *w_x, float *w_y) = 0;
    //判断是否位于地图内
    virtual inline bool map_to_world(const uint32_t &m_x, const uint32_t &m_y, float *w_x, float *w_y) = 0;
    virtual inline bool map_to_world(const float &m_x, const float &m_y, float *w_x, float *w_y)
    {
        //PRINT_INFO();
        *w_x = m_x * this->resolution() + this->origin_x();
        *w_y = m_y * this->resolution() + this->origin_y();
        return true;
    }
    virtual inline bool world_to_map(const float &w_x, const float &w_y, uint32_t *m_x, uint32_t *m_y) = 0;
    virtual inline bool world_to_map(const float &w_x, const float &w_y, float *m_x, float *m_y)
    {
        *m_x = (w_x - this->origin_x()) / this->resolution();
        *m_y = (w_y - this->origin_y()) / this->resolution();
        return true;
    }
    virtual inline bool is_world_position_in_map_bounds(const float &w_x, const float &w_y)
    {
        uint32_t m_x = 0;
        uint32_t m_y = 0;
        world_to_map(w_x, w_y, &m_x, &m_y);
        return is_position_in_map_bounds(m_x, m_y);
    }

    virtual inline bool is_position_in_map_bounds(const int32_t &mx, const int32_t &my)
    {
        if(mx < 0 || mx >= size_in_cells_x() || my < 0 || my >= size_in_cells_y())
        {
            return false;
        }
        return true;
    }

    /**
     * @brief  Given two map coordinates... compute the associated index
     * @param mx The x coordinate
     * @param my The y coordinate
     * @return The associated index
     */

    virtual inline uint32_t index(const uint32_t &mx, const uint32_t &my) const = 0;
    virtual inline void index_to_cells(uint32_t index, uint32_t *mx, uint32_t *my)= 0;
//    virtual uint8_t* map() const = 0;
    virtual inline uint32_t size_in_cells_x() const = 0;
    virtual inline uint32_t size_in_cells_y() const = 0;
    virtual inline float size_in_meters_x() const = 0;
    virtual inline float size_in_meters_y() const = 0;
    virtual inline float origin_x() const = 0;
    virtual inline float origin_y() const = 0;
    virtual inline float resolution() const = 0;

    virtual inline std::vector<VectorX2<float>> robot_footprint() = 0;

    virtual bool is_free(Pose<float> cur_pose)
    {
        return !this->map_collision_detect(cur_pose);
    }
    virtual uint8_t world_cost(const Pose<float> &cur_pose)
    {
        return this->world_cost(cur_pose.position.x, cur_pose.position.y);
    }

    virtual uint8_t world_cost(const float &wx, const float &wy)
    {
        uint32_t cost_value = obstacles_cost();
        uint32_t mx = 0;
        uint32_t my = 0;
        if(world_to_map(wx, wy, &mx, &my))
        {
            cost_value = cost(mx, my);
        }
        return cost_value;
    }

    //reture true means exist obstacle inside robot footprint
    virtual bool world_collision_detect(const Pose<float> &cur_pose)
    {
        uint32_t mx = (int)cur_pose.position.x;
        uint32_t my = (int)cur_pose.position.y;
        if(world_to_map(cur_pose.position.x, cur_pose.position.y, &mx, &my))
        {
            Pose<float> map_pos(mx, my);
            return map_collision_detect(map_pos);
        }
        PRINT_WARN("world pose out of map: world pose({:.1f}, {:.1f}), map pose({:d}, {:d})", cur_pose.position.x, cur_pose.position.y, (int)mx, (int)my);
        return true;
    }

    virtual bool map_collision_detect(const Pose<float> &cur_pose)
    {
        //因为地图上障碍物经过膨胀，所以只需要单点的cost值就可以
        //return this->get_cost(cur_pose.position.x, cur_pose.position.y);
        const uint8_t threshold = obstacles_cost()-3;
        if(this->cost(cur_pose.position.x, cur_pose.position.y) > threshold)
        {
            //PRINT_ERROR("cur pos exist obstacles");
            return true;
        }
        std::vector<VectorX2<float>> footprint = robot_footprint();
        const VectorX2<float> offset(cur_pose.position.x, cur_pose.position.y);
        float min_x = DBL_MAX;
        float max_x = 0;
        float min_y = DBL_MAX;
        float max_y = 0;

        float cos_heading_angle = cos(cur_pose.heading_angle);
        float sin_heading_angle = sin(cur_pose.heading_angle);
        float map_pose_x = 0;
        float map_pose_y = 0;

        for(int i = 0; i < footprint.size(); i++)
        {
            VectorX2<float> new_pt;
            new_pt.x = cur_pose.position.x + (footprint[i].x * cos_heading_angle - footprint[i].y * sin_heading_angle)/resolution();
            new_pt.y = cur_pose.position.y + (footprint[i].x * sin_heading_angle + footprint[i].y * cos_heading_angle)/resolution();
            footprint[i] = new_pt;
            min_x = std::min(min_x, footprint[i].x);
            max_x = std::max(max_x, footprint[i].x);
            min_y = std::min(min_y, footprint[i].y);
            max_y = std::max(max_y, footprint[i].y);
        }

        uint32_t range_min_x = uint32_t(std::max(min_x, 0.0f));
        uint32_t range_max_x = uint32_t(std::min(uint32_t(max_x), size_in_cells_x() - 1));
        uint32_t range_min_y = uint32_t(std::max(min_y, 0.0f));
        uint32_t range_max_y = uint32_t(std::min(uint32_t(max_y), size_in_cells_y() - 1));
        //PRINT_INFO("%f, %f", map_pose_x, map_pose_y);
        //PRINT_INFO("%f, %f, %f, %f", min_x , min_y, max_x, max_y);
        //uint8_t obstacle_cost = (uint8_t)p_map->obstacles_cost() - 2;
        uint8_t obstacle_cost = 99;
        for(uint32_t x_pos = range_min_x; x_pos <= range_max_x; ++x_pos)
        {
            for(uint32_t y_pos = range_min_y; y_pos <= range_max_y; ++y_pos)
            {
                //if(x_pos < p_map->size_in_cells_x() && y_pos < p_map->size_in_cells_y())
                {
                    if(cost(x_pos, y_pos) > obstacle_cost)
                    {
                        VectorX2<float> point((float(x_pos)), float (y_pos));
                        if(is_inside_robot_footprint(point, footprint))
                        {
                            return true;
                        }
                        //                            }
                    }
                }
            }
        }
        return false;
    }

    virtual bool collision_detect(const Pose<float> &cur_pose, const float &path_angle)
    {
//        return this->cost(cur_pose.position.x, cur_pose.position.y);
//        if(this->cost(cur_pose.position.x, cur_pose.position.y) >= 254)
//        {
//            PRINT_ERROR("cur pos exist obstacles");
//            return true;
//        }
        const uint8_t threshold = obstacles_cost() - 2;
        std::vector<VectorX2<float>> footprint = robot_footprint();
        const VectorX2<float> offset(cur_pose.position.x, cur_pose.position.y);
        float min_x = DBL_MAX;
        float max_x = 0;
        float min_y = DBL_MAX;
        float max_y = 0;
        for(int i = 0; i < footprint.size(); i++)
        {
            footprint[i] = rotate_2d(footprint[i], -cur_pose.heading_angle);
            footprint[i].x /= resolution();
            footprint[i].y /= resolution();
            footprint[i] = footprint[i] + offset;
            min_x = std::min(min_x, footprint[i].x);
            max_x = std::max(max_x, footprint[i].x);
            min_y = std::min(min_y, footprint[i].y);
            max_y = std::max(max_y, footprint[i].y);
        }
//        if(min_x < 0 || max_x > size_in_cells_x() || min_y < 0 || max_y > size_in_cells_y())
//        {
//            PRINT_DEBUG("cur pos: {:.1f}, {:.1f}  [{:.1f} {:.1f} {:.1f} {:.1f}] > [{:d}, {:d}]", cur_pose.position.x, cur_pose.position.y,
//                        min_x, max_x, min_y, max_y, size_in_cells_x(), size_in_cells_y());
//            PRINT_ERROR("range out of map bounds");
//            return true;
//        }
        uint32_t range_min_x = uint32_t(std::max(min_x, 0.0f));
        uint32_t range_max_x = uint32_t(std::min(uint32_t(max_x), size_in_cells_x()));
        uint32_t range_min_y = uint32_t(std::max(min_y, 0.0f));
        uint32_t range_max_y = uint32_t(std::min(uint32_t(max_y), size_in_cells_y()));
//        if(range_max_x < this->size_in_cells_x())
//        {
//            ++range_max_x;
//        }
//        if(range_max_x < this->size_in_cells_y())
//        {
//            ++range_max_y;
//        }

        for(uint32_t x_pos = range_min_x; x_pos < range_max_x; x_pos++)
        {
            for(uint32_t y_pos = range_min_y; y_pos < range_max_y; y_pos++)
            {
                if(is_position_in_map_bounds(x_pos, y_pos))
                {
                    if(this->cost(x_pos, y_pos) > threshold)
                    {
                        const static uint32_t offset_array[4][2] = {{0, 0}, {0, 1}, {1, 0}, {1, 1}};
//                        VectorX2<float> point;
//                        for(int i = 0; i < 4; i++)
//                        {
//                            point.x = float(x_pos + offset_array[i][0]);
//                            point.y = float(x_pos + offset_array[i][1]);
                            VectorX2<float> point((float(x_pos)), float (y_pos));
                            if(this->is_inside_robot_footprint(point, footprint))
                            {
                                //PRINT_DEBUG("  [{:d} {:d}] -> [{:d}, {:d}]", range_min_x, range_min_y, range_max_x, range_max_y);
                                float map_x;
                                float map_y;
                                map_to_world(x_pos, y_pos, &map_x, &map_y);
//                                PRINT_ERROR("obstacles in the car({:d}). map pos: {:d}, {:d} world pos({:.1f}, {:.1f})",
//                                            cost(x_pos, y_pos), x_pos, y_pos, map_x, map_y);
                                float obstacle_angle = atan2(y_pos - cur_pose.position.y, x_pos - cur_pose.position.x);
                                if(fabs(constraint_angle_r(obstacle_angle - path_angle, -M_PI, M_PI) > M_PI_2))
                                {
                                    return true;
                                }
                            }
//                        }
                    }
                }
            }
        }
        return false;
    }

    virtual bool is_inside_robot_footprint(const VectorX2<float> &point, const std::vector<VectorX2<float>> &footprint)
    {
        //return true;
        int counter = 0;
        bool is_inside = false;
        float xinters;
        VectorX2<float> p1;
        VectorX2<float> p2;

        p1 = footprint[0];
        int len = footprint.size();
        for(int i = 1; i < len+1; i++)
        {
           p2 = footprint[i % len];
           //is point on segment
           if(point.x <= std::max(p1.x, p2.x) && point.x >= std::min(p1.x, p2.x))
           {
                if(point.y <= std::max(p1.y, p2.y) && point.y >= std::min(p1.y, p2.y))
                {
                    if((point.x-p1.x)*(p2.y-p1.y)==(p2.x-p1.x)*(point.y-p2.y))
                    {
                        return true;
                    }
                }
           }
           if (point.y > std::min(p1.y, p2.y) && point.y <= std::max(p1.y, p2.y))
           {
               if (point.x <= std::max(p1.x, p2.x))
               {
                   if (p1.y != p2.y)
                   {
                       xinters = (point.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                       if (p1.x == p2.x || point.x <= xinters)
                       {
//                           counter++;
                            is_inside = !is_inside;
                       }

                   }
               }
           }
           p1 = p2;
       }

    //    if (counter % 2 == 0)
    //       return(OUTSIDE);
    //    else
    //       return(INSIDE);

        return is_inside;
    }
    virtual inline int8_t obstacles_cost() = 0;
    virtual inline int8_t no_information_cost() = 0;
    virtual inline uint8_t cost(const uint32_t &mx, const uint32_t &my) = 0;
    virtual inline uint8_t cost(const float &mx, const float &my)
    {
        uint32_t x = int(mx);
        uint32_t y = int(my);
        return cost(x, y);
    }
    virtual uint8_t robot_foot_print_map_cost(const Pose<float> &cur_pose)
    {
        const uint8_t threshold = obstacles_cost() - 1;
        std::vector<VectorX2<float>> footprint = robot_footprint();
        const VectorX2<float> offset(cur_pose.position.x, cur_pose.position.y);
        float min_x = DBL_MAX;
        float max_x = 0;
        float min_y = DBL_MAX;
        float max_y = 0;
        for(int i = 0; i < footprint.size(); i++)
        {
            footprint[i] = rotate_2d(footprint[i], -cur_pose.heading_angle);
            footprint[i].x /= resolution();
            footprint[i].y /= resolution();
            footprint[i] = footprint[i] + offset;
            min_x = std::min(min_x, footprint[i].x);
            max_x = std::max(max_x, footprint[i].x);
            min_y = std::min(min_y, footprint[i].y);
            max_y = std::max(max_y, footprint[i].y);
        }

        uint32_t range_min_x = uint32_t(std::max(min_x, 0.0f));
        uint32_t range_max_x = uint32_t(std::min(uint32_t(max_x), size_in_cells_x()));
        uint32_t range_min_y = uint32_t(std::max(min_y, 0.0f));
        uint32_t range_max_y = uint32_t(std::min(uint32_t(max_y), size_in_cells_y()));

        uint8_t cost_value = 0;
        for(uint32_t x_pos = range_min_x; x_pos < range_max_x; x_pos++)
        {
            for(uint32_t y_pos = range_min_y; y_pos < range_max_y; y_pos++)
            {
                if(is_position_in_map_bounds(x_pos, y_pos))
                {
                    if(this->cost(x_pos, y_pos) >= threshold)
                    {
                        const static uint32_t offset_array[4][2] = {{0, 0}, {0, 1}, {1, 0}, {1, 1}};
                        VectorX2<float> point;
                        for(int i = 0; i < 4; i++)
                        {
                            point.x = float(x_pos + offset_array[i][0]);
                            point.y = float(x_pos + offset_array[i][1]);
                            if(this->is_inside_robot_footprint(point, footprint))
                            {
                                cost_value = std::max(cost_value, cost((uint32_t)point.x, (uint32_t)point.y));
                            }
                        }
                    }
                }
            }
        }
        return cost_value;
    }
    std::string debug_info()
    {
        std::stringstream sstream;
        sstream << "map size x: " << size_in_cells_x() << " y: " << size_in_cells_y() << "\n"
            << "meter size x: " << size_in_meters_x() << " y: " << size_in_meters_x() << "\n"
            << "original point " << origin_x() << ", " << origin_y() << "\n"
            << "resolution " << resolution() << "\n";
        return sstream.str();
    }

    std::string map_to_um_json()
    {
        uint32_t num_points = 0;
        const std::vector<std::vector<int8_t>> map_data = std::move(this->map_data());
        const int obstacle_value = this->obstacles_cost();
        std::vector<std::vector<int>> json_map_data;
        const uint32_t x_size = map_data.size();
        const int origin_x = this->origin_x() * 1000;
        const int origin_y = this->origin_y() * 1000;
        const uint32_t resolution = this->resolution() * 1000;
        int min_x = origin_x;
        int min_y = origin_y;
        int max_x = (this->origin_x() + size_in_meters_x()) * 1000;
        int max_y = (this->origin_y() + size_in_meters_y()) * 1000;
        for(uint32_t x = 0; x != x_size; ++x)
        {
            bool is_exist_obstacles = false;
            std::vector<int> map_x_data;
            const uint32_t y_size = map_data[x].size();
            map_x_data.emplace_back(origin_x + x * resolution);
            for(uint32_t y = 0; y != y_size; ++y)
            {
                if(map_data[x][y] == obstacle_value)
                {
                    ++num_points;
                    is_exist_obstacles = true;
                    map_x_data.emplace_back(origin_y + y * resolution);
                }
            }
            if(is_exist_obstacles)
            {
                json_map_data.emplace_back(map_x_data);
            }
        }

        nlohmann::json j_out;
        j_out["Header"] = "";
        j_out["MapRes"] = int(resolution);

        char str[100];
        sprintf(str, "%d %d", max_x, max_y);
        j_out["MaxPose"] = str;
        sprintf(str, "%d %d", min_x, min_y);
        j_out["MinPose"] = str;
        j_out["NumLines"] = 0;
        j_out["NumPoints"] = num_points;
        j_out["ObsPoints"] = json_map_data;
        return j_out.dump();
    }
};

}
