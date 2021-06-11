#include "fmm_planner.h"
#include <stdio.h>
#include <cassert>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <mcheck.h>
#include <queue>
#include "common/print.h"
#include "common/time_keeper.h"
#include "common/common.h"

#include "fmm_planner/heap.h"

namespace bz_robot
{
namespace fmm_planner
{
static const float INF = std::numeric_limits<float>::infinity();
static const float N_INF = std::numeric_limits<float>::lowest();

static const char UNKNOWN = 0;
static const char KNOWN = 1;
static const char FRONT = 2;

static const float nine_fourths = 9.0 / 4.0;
static const float one_third = 1.0 / 3.0;


FMMPlanner::FMMPlanner()
{
    //todo 为什么对速度赋这样的值？？？？？？？？？？ 0XFF=0-255
    m_speed_list.resize(0XFF);
    for(int i = 0; i < 0XFF; ++i)
    {
        m_speed_list[i] = -1.0 - 0.1 * i;
    }
}

bool FMMPlanner::plan(std::shared_ptr<MapBase> p_map, Pose<float> pose_start, Pose<float> pose_goal,
                            const uint32_t &over_time_ms, std::vector<Pose<float> > *p_path)
{
    RECORD_TIME();
    mp_map = p_map;
    //std::swap(pose_start, pose_goal);
    m_start_pose = pose_start;
    m_goal_pose = pose_goal;
    bool is_find_path = false;
    p_path->clear();
    uint32_t map_start_pose_x;
    uint32_t map_start_pose_y;
    uint32_t map_goal_pose_x;
    uint32_t map_goal_pose_y;
    if(!mp_map->world_to_map(pose_start.position.x, pose_start.position.y, &map_start_pose_x, &map_start_pose_y))
    {
        return false;
    }

    if(!mp_map->world_to_map(pose_goal.position.x, pose_goal.position.y, &map_goal_pose_x, &map_goal_pose_y))
    {
        return false;
    }
    PRINT_DEBUG("start pose {:d}, {:d}", map_start_pose_x, map_start_pose_y);
    PRINT_DEBUG("goal pose {:d}, {:d}", map_goal_pose_x, map_goal_pose_y);
    //Pose<float> map_pose_goal(map_goal_pose_x, map_goal_pose_y, pose_goal.heading_angle);
    is_find_path = fmm_explore(mp_map, pose_start, pose_goal);


//    int s = 0;
//    uint32_t size = m_map_w * m_map_h;
//    std::cout << "fmm solution:" << std::endl;
//    for (int i = 0; i < size; i++)
//    {
//        printf("%2.2f\t", m_fmm_costmap[i]);
//        if (++s >= m_shape[1])
//        {
//            s = 0;
//            std::cout << std::endl;
//        }
//    }

    VectorX2<int> map_pose(map_start_pose_x, map_start_pose_y);

    flow_field(m_costmap, map_pose, p_path);

    return is_find_path;
}

std::vector<std::vector<float> > FMMPlanner::costmap()
{
    return m_costmap;
}

bool FMMPlanner::fmm_explore(std::shared_ptr<MapBase> p_map, const Pose<float> &pose_start, const Pose<float> &pose_goal)
{
    //RECORD_TIME();
    mp_map = p_map;
    bool is_find_path = false;
    uint32_t map_pose_x;
    uint32_t map_pose_y;
    uint32_t map_pose_start_x;
    uint32_t map_pose_start_y;
    //1.获取到起始点和目标点在栅格地图中的坐标值，并确定点是否在栅格地图的范围内
    if(!p_map->world_to_map(pose_start.position.x, pose_start.position.y, &map_pose_start_x, &map_pose_start_y))
    {
        return false;
    }
    p_map->world_to_map(pose_goal.position.x, pose_goal.position.y, &map_pose_x, &map_pose_y);
    //2.获取基本的地图参数，如长度，宽度，代价阈值，初始化全局各个栅格的代价值，以及各个栅格的状态量
    const uint8_t obstacle_value = p_map->obstacles_cost() - 1;
    uint32_t map_w = p_map->size_in_cells_x();
    uint32_t map_h = p_map->size_in_cells_y();
    m_map_w = map_w;
    m_map_h = map_h;
    m_fmm_costmap.assign(map_w * map_h, INF);
    m_flags.assign(map_w * map_h, UNKNOWN);
    //source为终点所在的栅格？？？？？？？？？？？？？
    uint32_t source = map_pose_x + map_pose_y * map_w;
    m_fmm_costmap[source] = 0;//按照FMM的波传导方式，目标点是波的起点，赋值为0
    const uint8_t ndim = 2;//维数
    uint32_t size = 1;
    m_shape[0] = m_map_h;
    m_shape[1] = m_map_w;
    //更新切换系数m_shift[0]=m_map_w，m_shift[1]=1
    for (int i = ndim - 1; i >= 0; i--)
    {
        m_shift[i] = size;
        size *= m_shape[i];
    }

    //PRINT_DEBUG("xs = {:d}", source);
//    []        //未定义变量.试图在Lambda内使用任何外部变量都是错误的.
//    [x, &y]   //x 按值捕获, y 按引用捕获.
//    [&]       //用到的任何外部变量都隐式按引用捕获
//    [=]       //用到的任何外部变量都隐式按值捕获
//    [&, x]    //x显式地按值捕获. 其它变量按引用捕获
//    [=, &z]   //z按引用捕获. 其它变量按值捕获
    auto heap_comp = [&](const uint32_t &e1, const uint32_t &e2){return m_fmm_costmap[e1] < m_fmm_costmap[e2];};
    Heap<decltype(heap_comp)> front(heap_comp, size);//front的size是栅格尺寸的大小
    front.push(source);
    std::vector<unsigned long> minima;
    while (!front.empty())
    {
        minima.clear();
        // 小端的第一个参数为代价值最小的栅格的编号
        minima.push_back(front.top());//minima的元素是代价值最小的网格编号,除非有多个元素代价大小相同，否则永远为1
        m_flags[front.top()] = KNOWN;// 将该点的属性标注为已知，后面不再加入搜索范围
        float value = m_fmm_costmap[front.top()];//记录下当前编号下的代价值
        //std::cout<<"***************************"<<std::endl;
        front.pop();
        //当front的前n个元素相等时，用这种方法记录代价值最小的代价值编号
        bool done = false;
        while (!done)
        {
            if (!front.empty() && m_fmm_costmap[front.top()] == value)
            {
                minima.push_back(front.top());
                m_flags[front.top()] = KNOWN;
                front.pop();
            }
            else
            {
              done = true;
            }
        }
        uint32_t x = 0;
        uint32_t y = 0;
        //遍历代价值最小的点的邻居点，将其中合适的点加入到front中，并在适当的条件下进行堆更新
        for (unsigned long x_i : minima)
        {
            unsigned long rem = x_i;
            for (int d = 0; d < ndim; d++)
            {
                // 1.获取点x_i的坐标值，第一个循环获取y,第二个循环获取x
                unsigned long dim_i = rem / m_shift[d];
                rem -= dim_i * m_shift[d];
                //2.获取xi在y(x)方向上的左邻居，对其进行检查，当栅格的状态不是KNOWN且不存在障碍物时，对其进行操作，否则就要舍弃
                uint32_t x_n = x_i - m_shift[d];
                y = x_n / map_w;
                x = x_n - y * map_w;
                if (dim_i > 0 && m_flags[x_n] != KNOWN
                    && p_map->cost(x, y) < obstacle_value)
                {
                    m_fmm_costmap[x_n] = solve_quadratic(x_n, &m_fmm_costmap);
                    // 当栅格的状态为未知时，就可以将点标记为FRONT，并将它放入front
                    if (m_flags[x_n] == UNKNOWN)
                    {
                        front.push(x_n);
                        m_flags[x_n] = FRONT;
                    }
                    // 如果x_n是FRONT，保证x_n位于最小堆中的合适位置
                    else
                        front.update(x_n);
                }
                //3.获取xi在y(x)方向上的右邻居，对其进行检查，当栅格的状态不是KNOWN且不存在障碍物时，对其进行操作，否则就要舍弃
                x_n = x_i + m_shift[d];
                y = x_n / map_w;
                x = x_n - y * map_w;
                if (dim_i < m_shape[d] - 1 && m_flags[x_n] != KNOWN
                    && p_map->cost(x, y) < obstacle_value)
                {
                    m_fmm_costmap[x_n] = solve_quadratic(x_n, &m_fmm_costmap);
                    // 当栅格的状态为未知时，就可以将点标记为FRONT，并将它放入front
                    if (m_flags[x_n] == UNKNOWN)
                    {
                        front.push(x_n);
                        m_flags[x_n] = FRONT;
                    }
                    // 保证x_n位于最小堆中的合适位置
                    else
                      front.update(x_n);
                }
            }
        }
    }

    m_costmap.resize(m_map_w);
    for(int i = 0; i < m_map_w; ++i)
    {
        m_costmap[i].resize(m_map_h);
    }
    for(int j = 0; j < m_map_h; ++j)
    {
        uint32_t base = j * m_map_w;
        for(int i = 0; i < m_map_w; ++i)
        {
            uint32_t x = base + i;
            m_costmap[i][j] = m_fmm_costmap[x];
        }
    }
    if(m_costmap[map_pose_start_x][map_pose_start_y] != INF)
    {
//        PRINT_DEBUG("FMM [{:d}]][{:d}] = {:2.2f}", map_pose_start_x, map_pose_start_y,
//                    m_costmap[map_pose_start_x][map_pose_start_y]);
        is_find_path = true;
    }
    return is_find_path;
}
//用来更新fmm_costmap,x为当前点的邻居点编号，tau为代价地图指针
inline float FMMPlanner::solve_quadratic(const uint32_t &x, std::vector<float> *tau)
{
    const uint8_t ndim = 2;
    const uint8_t order = 2;
    unsigned long rem = x;
    for (int d = 0; d < ndim; d++)
    {
        // 求解出x在当前坐标系中的坐标位置 m_shift[0] = map_x,m_shift[1] = 1;
        // 第一次循环得到纵坐标，第二次得到横坐标
        unsigned long dim_i = rem / m_shift[d];
        rem -= dim_i * m_shift[d];

        float tau_n = INF;
        char direction = 0;
        // choose the direction considering the non-factored first order approximation (tau_i-1 < tau_i+1 -> backward direction)
        // 找到状态为KNOWN的左邻居，获取左邻居对应的fmm代价地图值
        if (dim_i > 0 && m_flags[x - m_shift[d]] == KNOWN)
        {
            tau_n = (*tau)[x - m_shift[d]];
            direction = -1;
        }
        // 找到状态为KNOWN的右邻居，获取右邻居对应的fmm代价地图值
        if (dim_i < m_shape[d] - 1 && m_flags[x + m_shift[d]] == KNOWN && (*tau)[x + m_shift[d]] < tau_n)
        {
            tau_n = (*tau)[x + m_shift[d]];
            direction = 1;
        }

        // 将获取到的有效网格加入计算
        if (direction != 0)
        {
            m_skipped[d] = false;
            // 2阶 && 在方向上处于边界内，从边界往里膨胀一层 && 当前方向上往外扩展两层栅格是处于已知状态的 && 当前栅格的代价值大于向外扩展两层后的代价值
            if (order >= 2
                && ((direction == -1 && dim_i > 1) || (direction == 1 && dim_i < m_shape[d] - 2))
                && m_flags[x + 2 * direction * m_shift[d]] == KNOWN
                && tau_n > (*tau)[x + 2 * direction * m_shift[d]])
            {
                //alpha_sq[d] = nine_fourths * dx_sq_inv[d];
                alpha_sq[d] = nine_fourths;// 9/4
                m_beta[d] = one_third * (4.0 * tau_n - (*tau)[x + 2 * direction * m_shift[d]]);
            }
            // otherwise fall back to first order
            else
            {
                //alpha_sq[d] = dx_sq_inv[d];
                alpha_sq[d] = 1.0;
                m_beta[d] = tau_n;
            }
        }
        else
            m_skipped[d] = true;
    }


    //float c_base = -1.0 / pow(this->m_c[x], 2);
    //假设每个点走过的代价是一样的
    float c_base = -1.0;
    //从全局地图代价值更新c_base
    uint32_t map_x = x % m_map_w;
    uint32_t map_y = x / m_map_w;
    c_base = m_speed_list[mp_map->cost(map_x,map_y)];

    float a, b, c;
    float disc;//b2 - 4ac

    // the currently biggest "beta" and it's dimension
    float biggest;
    int biggest_d;

    do
    {
        a = 0.0, b = 0.0;
        c = c_base;

        biggest = N_INF;//初始化为负无穷
        biggest_d = -1;

        for (int d = 0; d < ndim; d++)
        {
            if (!m_skipped[d])
            {
                a += alpha_sq[d];
                b -= 2.0 * alpha_sq[d] * m_beta[d];
                c += alpha_sq[d] * pow(m_beta[d], 2);

                if (m_beta[d] > biggest)
                {
                    biggest_d = d;
                    biggest = m_beta[d];
                }
            }
        }

        if (biggest_d == -1)
            throw std::runtime_error("Negative discriminant in solve_quadratic.");

        disc = pow(b, 2) - 4.0 * a * c;
        m_skipped[biggest_d] = true;
    } while (disc < 0);

    // a is always positive, so the '+' solution is larger (causality)
    return (-b + sqrt(disc)) / (2.0 * a);
}



void FMMPlanner::flow_field(const std::vector<std::vector<float> > &costmap,
                                  const VectorX2<int> &start_pose,
                                  std::vector<Pose<float> > *p_path)
{
    RECORD_TIME();
//    printf("\n");

//    for(int j = 0; j < m_map_h; ++j)
//    {
//        for(int i = 0; i < m_map_w; ++i)
//        {
//            printf("%2.2f\t", i, j, costmap[i][j]);
//        }
//        printf("\n");
//    }
    PRINT_DEBUG("start pose: {:d},{:d}", start_pose.x, start_pose.y);
    PRINT_INFO("costmap[{:d}][{:d}] = %.2f", start_pose.x, start_pose.y,
               costmap[start_pose.x][start_pose.y]);
    VectorX2<int> cur_pose = start_pose;
    VectorX2<int> prev_pose = start_pose;
    VectorX2<int> next_pose = start_pose;
    do
    {
        float min_cost = costmap[next_pose.x][next_pose.y];
        int min_x = std::max(0, cur_pose.x - 1);
        int max_x = std::min((int)costmap.size(), cur_pose.x + 2);
        int min_y = std::max(0, cur_pose.y - 1);
        int max_y = std::min((int)costmap[0].size(), cur_pose.y + 2);
        //printf("cur pos: {:d}, {:d}\n", cur_pose.x, cur_pose.y);
        for(int x = min_x; x < max_x; ++x)
        {
            for(int y = min_y; y < max_y; ++y)
            {
                if(x == cur_pose.x && y == cur_pose.y)
                {
                    continue;
                }
                if(x == prev_pose.x && y == prev_pose.y)
                {
                    continue;
                }
                //PRINT_DEBUG("[{:d}][{:d}] = %.2f", x, y, costmap[x][y]);
                if(!std::isinf( costmap[x][y])&&
                    min_cost > costmap[x][y])
                {
                    min_cost = costmap[x][y];
                    next_pose.x = x;
                    next_pose.y = y;
                }
            }
        }
        prev_pose = cur_pose;
        cur_pose = next_pose;
        PRINT_INFO("costmap[{:d}][{:d}] = %.2f", next_pose.x, next_pose.y, costmap[next_pose.x][next_pose.y]);
//        if(costmap[next_pose.x][next_pose.y] == min_cost)
//        {
//            break;
//        }

        Pose<float> pose;
        mp_map->map_to_world((uint32_t)next_pose.x, (uint32_t)next_pose.y, &pose.position.x, &pose.position.y);
        p_path->emplace_back(pose);
    }
    while(costmap[cur_pose.x][cur_pose.y] > 0);
    //smooth_path(mp_map, p_path);
    (*p_path)[0] = m_start_pose;
    p_path->back() = m_goal_pose;
}
}; // namespace fmm_planner
}
