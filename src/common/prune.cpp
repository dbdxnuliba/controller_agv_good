#include "prune.h"
#include "common/print.h"
#include "modules/map/map_base.h"
#include <cassert>

namespace bz_robot
{
Prune::Prune():
    m_begin_index(0),
    m_last_begin_index(0)
{

}

void Prune::run(std::shared_ptr<MapBase> p_map, const Pose<float> &cur_pose, std::vector<Pose<float> > *p_path, float padding)
{
    assert(p_path != nullptr);

//    for(int i = 0; i < path.ize(); i++)
//    {
//        PRINT_DEBUG("index: {:d} pos: {:.2f}, {:.2f}, {:.2f}", i,
//        (*p_path)[i].position.x, (*p_path)[i].position.y,
//        (*p_path)[i].heading_angle * M_1_PI * 180.0);
//    }
    //PRINT_DEBUG("cur_pose: {:.1f}, {:.1f}", cur_pose.position.x, cur_pose.position.y);

    if(p_path->size() < 3)
    {
        return;
    }
    //PRINT_DEBUG("({:.1f}, {:.1f}) => ({:.1f}, {:.1f})", p_path->front().position.x, p_path->front().position.y, p_path->back().position.x, p_path->back().position.y);
    float dis = DBL_MAX;
    int start_index = 0;

    for(int i = 0; i < p_path->size(); ++i)
    {
        float dx = (*p_path)[i].position.x - cur_pose.position.x;
        float dy = (*p_path)[i].position.y - cur_pose.position.y;
        float distance = hypot(dx, dy);
        float angle_diff = fabs(constraint_angle_r((*p_path)[i].heading_angle - cur_pose.heading_angle, -M_PI, M_PI));

        //PRINT_DEBUG("index: {:d} pos: {:.1f}, {:.1f}", i, (*p_path)[i].position.x, (*p_path)[i].position.y);
        if(distance <= dis)
        {
            dis = distance;
        }
        else
        {
            if(angle_diff < M_PI_2)
            {
                start_index = i - 1;
                break;
            }
        }
    }
    start_index = std::max(0, start_index);

    std::vector<Pose<float>> path = *p_path;
    // for(int i = 0; i < path.size(); i++)
    // {
    //     PRINT_DEBUG(" {:d}:({:.1f}, {:.1f}) {:.1f}", i, path[i].position.x, path[i].position.y, path[i].heading_angle * 180 / M_PI);
    // }
    uint32_t target_index = 0;
    float min_distance = DBL_MAX;

    // int min_index = 0;
    for(target_index = m_begin_index; target_index < path.size(); ++target_index)
    {
        float dx = cur_pose.position.x - path[target_index].position.x;
        float dy = cur_pose.position.y - path[target_index].position.y;

        float distance = pow(dx, 2) + pow(dy, 2);
        //PRINT_DEBUG("min_distance = %f, diatance = %f", min_distance, distance);
        if(distance > min_distance)
        {
            break;
        }
        else
        {
            min_distance = distance;
            m_begin_index = target_index;
        }
    }
//    PRINT_DEBUG("start_index = {:d}", start_index);
//    PRINT_DEBUG("m_begin_index = {:d}", m_begin_index);
    //for test
    //m_begin_index = start_index;
    /**
     * 在上一步寻找与当前点最近的点中，这个点可能是即将到达的点，也可能是已经经过的点
     * 这边通过计算向量的角度，统一调整为已经经过的点
     */
    float vector_s_x = 0;
    float vector_s_y = 0;
    if(m_begin_index < path.size())
    {
        vector_s_x = path[m_begin_index+1].position.x - path[m_begin_index].position.x;
        vector_s_y = path[m_begin_index+1].position.y - path[m_begin_index].position.y;
        float cur_x = cur_pose.position.x - path[m_begin_index].position.x;
        float cur_y = cur_pose.position.y - path[m_begin_index].position.y;
        if(vector_s_x * cur_x + vector_s_y * cur_y < 0)
        {
            if(m_begin_index > 0)
            {
                --m_begin_index;
            }
            if(m_begin_index < m_last_begin_index)
            {
                //PRINT_DEBUG("m_begin_index = {}, m_last_begin_index = {}", m_begin_index, m_last_begin_index);
                m_begin_index = m_last_begin_index;
            }
        }
    }
    else
    {
        m_begin_index = path.size() - 1;
    }

    target_index = m_begin_index;
    m_last_begin_index = m_begin_index;
    //PRINT_DEBUG("start_index = {:d}, {:.2f}, {:.2f}", m_begin_index, path[m_begin_index].position.x, path[m_begin_index].position.y);
    p_path->clear();
    //p_path->emplace_back(cur_pose);
    for(int i = m_begin_index; i < path.size(); i++)
    {
//            if(path[i].position.x < cur_pose.position.x - p_map->size_in_meters_x() * 0.5 ||
//                    path[i].position.x > cur_pose.position.x + p_map->size_in_meters_x() * 0.5 ||
//                    path[i].position.y < cur_pose.position.y - p_map->size_in_meters_y() * 0.5 ||
//                    path[i].position.y > cur_pose.position.y + p_map->size_in_meters_y() * 0.5)
//            {
//                break;
//            }
        if(path[i].position.x < p_map->origin_x() + padding||
                path[i].position.x > p_map->origin_x() + p_map->size_in_meters_x() - padding ||
                path[i].position.y < p_map->origin_y() + padding||
                path[i].position.y > p_map->origin_y() + p_map->size_in_meters_y() - padding)
        {
//            PRINT_DEBUG("{:.3f}, {:.3f}, {:.3f}, {:.3f}", p_map->origin_x(), p_map->origin_x() + p_map->size_in_meters_x(),
//                        p_map->origin_y(), p_map->origin_y() + p_map->size_in_meters_y());
            break;
        }
        else
        {
            p_path->emplace_back(path[i]);
        }
    }


//    if(p_path->size() > 0)
//    {
//        (*p_path)[0] = cur_pose;
//    }

//     printf("\n");
//     for(int i = 0; i < p_path->size(); i++)
//     {
//         PRINT_DEBUG(" {:d}:({:.1f}, {:.1f}) {:.1f}", i, (*p_path)[i].position.x, (*p_path)[i].position.y, (*p_path)[i].heading_angle * 180 / M_PI);
//     }
//     PRINT_DEBUG("({:.1f}, {:.1f}) => ({:.1f}, {:.1f})", p_path->front().position.x, p_path->front().position.y, p_path->back().position.x, p_path->back().position.y);

}

void Prune::reset()
{
    //PRINT_DEBUG("reset\n\n");
    m_begin_index = 0;
    m_last_begin_index = 0;
}

int Prune::find_closest_index_in_path(const Pose<float> &cur_pose, std::vector<Pose<float>> &path)
{
    //PRINT_DEBUG("cur_pose: {:.1f}, {:.1f}", cur_pose.position.x, cur_pose.position.y);

    //PRINT_DEBUG("({:.1f}, {:.1f}) => ({:.1f}, {:.1f})", p_path->front().position.x, p_path->front().position.y, p_path->back().position.x, p_path->back().position.y);
    float dis = DBL_MAX;
    int start_index = 0;

    for(size_t i = 0; i < path.size(); ++i)
    {
        float dx = (path)[i].position.x - cur_pose.position.x;
        float dy = (path)[i].position.y - cur_pose.position.y;
        float distance = hypot(dx, dy);
        float angle_diff = fabs(constraint_angle_r((path)[i].heading_angle - cur_pose.heading_angle, -M_PI, M_PI));

        //PRINT_DEBUG("index: {:d} pos: {:.1f}, {:.1f}", i, (*p_path)[i].position.x, (*p_path)[i].position.y);
        if(distance <= dis)
        {
            dis = distance;
        }
        else
        {
            if(angle_diff < M_PI_2)
            {
                start_index = i - 1;
                break;
            }
        }
    }
    start_index = std::max(0, start_index);

//    std::vector<Pose<float>> path = *p_path;
    // for(int i = 0; i < path.size(); i++)
    // {
    //     PRINT_DEBUG(" {:d}:({:.1f}, {:.1f}) {:.1f}", i, path[i].position.x, path[i].position.y, path[i].heading_angle * 180 / M_PI);
    // }
    uint32_t target_index = 0;
    float min_distance = DBL_MAX;

    // int min_index = 0;
    for(target_index = m_begin_index; target_index < path.size(); ++target_index)
    {
        float dx = cur_pose.position.x - path[target_index].position.x;
        float dy = cur_pose.position.y - path[target_index].position.y;

        float distance = pow(dx, 2) + pow(dy, 2);
        //PRINT_DEBUG("min_distance = %f, diatance = %f", min_distance, distance);
        if(distance > min_distance)
        {
            break;
        }
        else
        {
            min_distance = distance;
            m_begin_index = target_index;
        }
    }
//    PRINT_DEBUG("start_index = {:d}", start_index);
//    PRINT_DEBUG("m_begin_index = {:d}", m_begin_index);
    //for test
    //m_begin_index = start_index;
    /**
     * 在上一步寻找与当前点最近的点中，这个点可能是即将到达的点，也可能是已经经过的点
     * 这边通过计算向量的角度，统一调整为已经经过的点
     */
    float vector_s_x = 0;
    float vector_s_y = 0;
    if(m_begin_index > 0)
    {
        vector_s_x = path[m_begin_index].position.x - path[m_begin_index-1].position.x;
        vector_s_y = path[m_begin_index].position.y - path[m_begin_index-1].position.y;
        float cur_x = cur_pose.position.x - path[m_begin_index].position.x;
        float cur_y = cur_pose.position.y - path[m_begin_index].position.y;
        if(vector_s_x * cur_x + vector_s_y * cur_y < 0)
        {
            --m_begin_index;
            if(m_begin_index < m_last_begin_index)
            {
                m_begin_index = m_last_begin_index;
            }
        }
    }
    m_last_begin_index = m_begin_index;
    return (int)m_begin_index;
}
}
