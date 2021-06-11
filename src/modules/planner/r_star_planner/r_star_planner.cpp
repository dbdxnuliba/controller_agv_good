#include "r_star_planner.h"
#include <stdio.h>
#include <cassert>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <mcheck.h>
#include <queue>
#include <list>
#include <limits>
#include <map>
#include <unordered_map>
#include "common/print.h"
#include "common/time_keeper.h"
#include "common/common.h"
//#include "path_smoother.h"
//for test
#include "common/pnm_image.h"


namespace bz_robot
{


namespace r_star_planner
{

RStarPlanner::RStarPlanner()
{
//    mp_path_smooth = new PathSmoother();
//    mp_path_smooth->set_smooth_times(500);
    m_fmm_planner = fmm_planner::FMMPlanner();
}

bool RStarPlanner::plan(std::shared_ptr<MapBase> p_map, Pose<float> pose_start, Pose<float> pose_goal, std::vector<Pose<float> > *p_path)
{
    //RECORD_TIME();
    mp_map = p_map;
    m_start_pose = pose_start;
    m_goal_pose = pose_goal;
    static const uint32_t max_iterators = 1000;
    uint32_t iterators = 0;
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
    Pose<float> map_pose_goal(map_goal_pose_x, map_goal_pose_y, pose_goal.heading_angle);


    //is_find_path = fast_wave_front(pose_start, pose_goal, p_path);
    //is_find_path = directed_wave_front(pose_start, pose_goal, p_path);
    is_find_path = directed_astar2(pose_start, pose_goal, p_path);

    //is_find_path = sorted_wave_front(pose_start, pose_goal, p_path);
    //is_find_path = sorted_wave_front(pose_start, pose_goal, p_path);
    return is_find_path;
}

void RStarPlanner::wave_front_explore(const Pose<float> &start_pose, const Pose<float> &goal_pose)
{
    //RECORD_TIME();
    const int8_t obstacle_value = mp_map->obstacles_cost()-1;
    uint32_t map_w = mp_map->size_in_cells_x();
    uint32_t map_h = mp_map->size_in_cells_y();
    m_wave_explore_costmap.resize(map_w);

    for(uint32_t i = 0; i < map_w; ++i)
    {
        m_wave_explore_costmap[i].assign(map_h, -1);
    }

    uint32_t map_pose_x;
    uint32_t map_pose_y;
    mp_map->world_to_map(goal_pose.position.x, goal_pose.position.y, &map_pose_x, &map_pose_y);
    m_wave_explore_costmap[map_pose_x][map_pose_y] = 0;
    //std::map<uint32_t, int> wave;
    //uint32_t index = mp_map->index(map_pose_x, map_pose_y);
    //int cost = 0;
    //wave[index] = cost;
    const static int pos_array[4][2] = {{1, 0}, {0, 1},  {-1, 0}, {0, -1}};
    //static int pos_array[8][2] = {{1, 0}, {0, 1},  {-1, 0}, {0, -1}, {1,1}, {-1, 1}, {-1, -1}, {1, -1}};
    const uint32_t array_size = sizeof(pos_array) / sizeof(pos_array[0]);
    std::queue<VectorX2<int>> qu;
    qu.push(VectorX2<int>(map_pose_x, map_pose_y));
    uint32_t it = 0;
    uint32_t map_pose_start_x;
    uint32_t map_pose_start_y;
    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_start_x, &map_pose_start_y);
    //bool is_find_path = false;
    while(!qu.empty())
    {

        VectorX2<int> map_pose = qu.front();
        qu.pop();
        //explore wave
        for(int i = 0; i < array_size; ++i)
        {
            ++it;
            VectorX2<int> next_map_pose;
            next_map_pose.x = map_pose.x + pos_array[i][0];
            next_map_pose.y = map_pose.y + pos_array[i][1];

            if(next_map_pose.x < map_w && next_map_pose.y < map_h)
            {
                if(m_wave_explore_costmap[next_map_pose.x][next_map_pose.y] == -1)
                {
                    int map_cost = (int)mp_map->cost((uint32_t)next_map_pose.x, (uint32_t)next_map_pose.y);

                    if(map_cost < obstacle_value)
                    {
                        qu.push(next_map_pose);
                        m_wave_explore_costmap[next_map_pose.x][next_map_pose.y] = m_wave_explore_costmap[map_pose.x][map_pose.y] + 1;
                    }
                }
            }
        }
    }
    PRINT_DEBUG("wave explore it = {}", it);
    //return is_find_path;
}

void RStarPlanner::wave_front_explore_3d(const Pose<float> &start_pose, const Pose<float> &goal_pose)
{
    //RECORD_TIME();
    typedef std::array<uint32_t, 4> Cell;
    const uint32_t I_X = 0;
    const uint32_t I_Y = 1;
    const uint32_t I_HEADING = 2;
    const uint32_t I_COST = 3;
    const int8_t obstacle_value = mp_map->obstacles_cost()-1;
    uint32_t map_w = mp_map->size_in_cells_x();
    uint32_t map_h = mp_map->size_in_cells_y();
    std::vector<std::vector<std::vector<int>>> wave_front_3d_costmap;
    {
        RECORD_TIME("allocate memory");
        wave_front_3d_costmap.resize(8);
        for(uint32_t i = 0; i < 8; ++i)
        {
            wave_front_3d_costmap[i].resize(map_w);
            for(uint32_t j = 0; j < map_w; ++j)
            {
                wave_front_3d_costmap[i][j].assign(map_h, -1);
            }
        }
    }
    uint32_t map_pose_x;
    uint32_t map_pose_y;
    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_x, &map_pose_y);
    uint32_t header_index = ((int)round(constraint_angle_r(start_pose.heading_angle) / M_PI_4)) % 8;

    wave_front_3d_costmap[header_index][map_pose_x][map_pose_y] = 0;

    static int move_to_direct[8][3][2] =
    {
        {{1,0}, {1, -1}, {1,1}},
        {{1,1}, {1,0}, {0, 1}},
        {{0,1}, {1,1}, {-1, 1}},
        {{-1,1}, {0,1}, {-1, 0}},
        {{-1,0}, {-1,1}, {-1, -1}},
        {{-1,-1}, {-1,0}, {0, -1}},
        {{0,-1}, {-1,-1}, {1, -1}},
        {{1,-1}, {0,-1}, {1, 0}},
    };
    static uint32_t move_direct_index[8][3] =
    {
        {0, 7, 1},
        {1, 0, 2},
        {2, 1, 3},
        {3, 2, 4},
        {4, 3, 5},
        {5, 4, 6},
        {6, 5, 7},
        {7, 6, 0},
    };

    std::queue<Cell> qu;
    qu.push(Cell{map_pose_x, map_pose_y, header_index, 0});

    //bool is_find_path = false;

    uint32_t it = 0;
    while(!qu.empty())
    {

        Cell map_pose = qu.front();
        qu.pop();

        //explore wave
        for(int i = 0; i < 3; ++i)
        {
            ++it;
            Cell next_map_pose;
            next_map_pose[I_X] = map_pose[I_X] + move_to_direct[map_pose[I_HEADING]][i][0];
            next_map_pose[I_Y] = map_pose[I_Y] + move_to_direct[map_pose[I_HEADING]][i][1];
            next_map_pose[I_HEADING] = move_direct_index[map_pose[I_HEADING]][i];

            if(next_map_pose[I_X] < map_w && next_map_pose[I_Y] < map_h)
            {
                if(wave_front_3d_costmap[next_map_pose[I_HEADING]][next_map_pose[I_X]][next_map_pose[I_Y]] == -1)
                {
                    int map_cost = (int)mp_map->cost((uint32_t)next_map_pose[I_X], (uint32_t)next_map_pose[I_Y]);

                    if(map_cost < obstacle_value)
                    {
                        int cost = wave_front_3d_costmap[map_pose[I_HEADING]][map_pose[I_X]][map_pose[I_Y]] + 1;
                        wave_front_3d_costmap[next_map_pose[I_HEADING]][next_map_pose[I_X]][next_map_pose[I_Y]] = cost;
                        qu.push(next_map_pose);
                    }
                }
            }
        }
    }
    PRINT_DEBUG("wave explore it = {}", it);
    //return true;
}


bool RStarPlanner::directed_wave_front(const Pose<float> &start_pose, const Pose<float> &goal_pose, std::vector<Pose<float> > *p_path)
{
    RECORD_TIME();
    class Node
    {
    public:
        Node(int mcost=-1):cost(mcost), step(0), is_closed(false), is_visited(false), path_cost(-1),
            p_next_node(nullptr), p_fathers({nullptr,nullptr,nullptr,nullptr,nullptr,nullptr}){};
    public:
        int cost;
        int step = 0;
        int x = 0;
        int y = 0;
        int header = 0;
        bool is_closed = false;
        bool is_visited = false;
        float path_cost = -1;
        Node* p_fathers[6];
        Node* p_next_node = nullptr;
    };

    typedef std::array<uint32_t, 4> Cell;
    const uint32_t I_X = 0;
    const uint32_t I_Y = 1;
    const uint32_t I_HEADING = 2;
    const uint32_t I_STEP = 3;
    const int8_t obstacle_value = mp_map->obstacles_cost()-1;
    uint32_t map_w = mp_map->size_in_cells_x();
    uint32_t map_h = mp_map->size_in_cells_y();
    const float r = 0.75 / tan(degree_to_radian(20));
    const int sub_step = ceil(r * tan(M_PI_4 * 0.5)*2 / mp_map->resolution());
    //const int half_sub_step = std::max(1, (int)ceil(r * tan(M_PI_4 * 0.5) / mp_map->resolution()));
    const int half_sub_step = std::max(1, (int)round(sub_step * 0.5));
    PRINT_DEBUG("sub step = {:d}, half_sub_step = {:d}, sub_step * 0.5 = {}", sub_step, half_sub_step, (sub_step * 0.5));

    std::vector<std::vector<std::vector<std::vector<Node>>>> wave_4d_node;
    //std::vector<std::vector<std::vector<int>>> wave_front_3d_costmap;
    {
        RECORD_TIME("allocate memory");
        //wave_front_3d_costmap.resize(8);
        wave_4d_node.resize(half_sub_step+1);
        for(uint32_t ss = 0; ss < half_sub_step+1; ++ss)
        {
            wave_4d_node[ss].resize(8);
            for(uint32_t i = 0; i < 8; ++i)
            {
                //wave_front_3d_costmap[i].resize(map_w);
                wave_4d_node[ss][i].resize(map_w);
                for(uint32_t j = 0; j < map_w; ++j)
                {
                    //wave_front_3d_costmap[i][j].assign(map_h, -1);
                    wave_4d_node[ss][i][j].resize(map_h, Node(-1));
                }
            }
        }

    }

    uint32_t map_pose_x;
    uint32_t map_pose_y;

    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_x, &map_pose_y);
    const uint32_t map_pose_start_x = map_pose_x;
    const uint32_t map_pose_start_y = map_pose_y;
    const uint32_t start_header_index = ((int)round(constraint_angle_r(start_pose.heading_angle) / M_PI_4)) % 8;
    wave_4d_node[1][start_header_index][map_pose_start_x][map_pose_start_y].cost = 0;
    mp_map->world_to_map(goal_pose.position.x, goal_pose.position.y, &map_pose_x, &map_pose_y);
    const uint32_t map_pose_goal_x = map_pose_x;
    const uint32_t map_pose_goal_y = map_pose_y;
    uint32_t goal_header_index = ((int)round(constraint_angle_r(goal_pose.heading_angle) / M_PI_4)) % 8;


    /*
     *  3   2   1
     *  4   *   0
     *  5   6   7
     */
    static int move_to_direct[8][3][2] =
    {
        {{1,0}, {1, -1}, {1,1}},
        {{1,1}, {1,0}, {0, 1}},
        {{0,1}, {1,1}, {-1, 1}},
        {{-1,1}, {0,1}, {-1, 0}},
        {{-1,0}, {-1,1}, {-1, -1}},
        {{-1,-1}, {-1,0}, {0, -1}},
        {{0,-1}, {-1,-1}, {1, -1}},
        {{1,-1}, {0,-1}, {1, 0}},
    };
    static uint32_t move_direct_index[8][3] =
    {
        {0, 7, 1},
        {1, 0, 2},
        {2, 1, 3},
        {3, 2, 4},
        {4, 3, 5},
        {5, 4, 6},
        {6, 5, 7},
        {7, 6, 0},
     };

    std::queue<Cell> qu;
    qu.push(Cell{map_pose_start_x, map_pose_start_y, start_header_index, 1});

    bool is_find_path = false;
    Cell map_pose;
    uint32_t it = 0;
    Node *p_cur_node = &wave_4d_node[1][start_header_index][map_pose_start_x][map_pose_start_y];
    p_cur_node->x = map_pose_start_x;
    p_cur_node->y = map_pose_start_y;
    p_cur_node->header = start_header_index;
    Node *p_next_node = nullptr;
    p_path->clear();
    std::vector<Pose<float>> explore_point;

    while(!qu.empty())
    {

        map_pose = qu.front();
        qu.pop();
        //explore wave
        p_cur_node = &wave_4d_node[map_pose[I_STEP]][map_pose[I_HEADING]][map_pose[I_X]][map_pose[I_Y]];
        p_cur_node->x = map_pose[I_X];
        p_cur_node->y = map_pose[I_Y];
        p_cur_node->header = map_pose[I_HEADING];
        if(p_cur_node->is_closed)
        {
            continue;
        }
        else
        {
            p_cur_node->is_closed = true;
        }
//        printf("{:d}, {:d}, {:d}\n", map_pose[I_X], map_pose[I_Y],
//               map_pose[I_HEADING]);
        Pose<float> pose;
        mp_map->map_to_world(map_pose[I_X], map_pose[I_Y],
                             &pose.position.x, &pose.position.y);
        explore_point.push_back(pose);

        if(map_pose[I_X] == map_pose_goal_x && map_pose[I_Y] == map_pose_goal_y &&
            goal_header_index == map_pose[I_HEADING] && map_pose[I_STEP] >= half_sub_step)
        {
            is_find_path = true;
            PRINT_DEBUG("find path!!!");
        }


        for(int i = 0; i < 3; ++i)
        {
            if(i > 0 && map_pose[I_STEP] < half_sub_step)
            {
//                PRINT_DEBUG("{:d}, {:d}, {:d} wang to turn to {:d}, skip this action", map_pose[I_X],map_pose[I_Y], map_pose[I_HEADING],
//                            move_direct_index[map_pose[I_HEADING]][i]);
                break;
            }
            ++it;
            Cell next_map_pose;
            next_map_pose[I_X] = map_pose[I_X] + move_to_direct[map_pose[I_HEADING]][i][0];
            next_map_pose[I_Y] = map_pose[I_Y] + move_to_direct[map_pose[I_HEADING]][i][1];
            next_map_pose[I_HEADING] = move_direct_index[map_pose[I_HEADING]][i];
            if(next_map_pose[I_X] < map_w && next_map_pose[I_Y] < map_h)
            {
                next_map_pose[I_STEP] = 1;
                if(i == 0)
                {
                    next_map_pose[I_STEP] = std::min((int)map_pose[I_STEP] + 1, half_sub_step);
                }
                p_next_node = &wave_4d_node[next_map_pose[I_STEP]][next_map_pose[I_HEADING]][next_map_pose[I_X]][next_map_pose[I_Y]];

                //if(p_next_node->cost == -1 || p_next_node->p_fathers[i] == nullptr)
                //if(p_next_node->cost == -1)
                {
                    int map_cost = (int)mp_map->cost((uint32_t)next_map_pose[I_X], (uint32_t)next_map_pose[I_Y]);
                    if(map_cost < obstacle_value)
                    {
                        int cost = p_cur_node->cost + 1;
                        p_next_node->cost = cost;
                        if(map_pose[I_STEP] == half_sub_step)
                        {
                            p_next_node->p_fathers[i] = p_cur_node;
                            //PRINT_DEBUG("p_next_node->p_fathers[{:d}] = %p", i, p_next_node->p_fathers[i]);
                        }
                        else
                        {
                            p_next_node->p_fathers[i+3] = p_cur_node;
                            //PRINT_DEBUG("p_next_node->p_fathers[{:d}] = %p", i+3, p_next_node->p_fathers[i+3]);
                        }
                        p_next_node->step = next_map_pose[I_STEP];
                        qu.push(next_map_pose);
//                        PRINT_DEBUG("{:d}, {:d}, {:d}, {:d} <=(father) {:d}, {:d}, {:d}, {:d}",
//                                    next_map_pose[I_X], next_map_pose[I_Y], next_map_pose[I_HEADING], next_map_pose[I_STEP],
//                                    map_pose[I_X], map_pose[I_Y], map_pose[I_HEADING], map_pose[I_STEP]);
//                        p_next_node->x = next_map_pose[I_X];
//                        p_next_node->y = next_map_pose[I_Y];
//                        p_next_node->header = next_map_pose[I_HEADING];
                        //path_map[next_map_pose] = map_pose;
                    }
                }
//                else
//                {
//                    int next_step = 0;
//                    switch (i)
//                    {
//                    case 0:
//                        if(p_next_node->p_s_father == nullptr)
//                        {
//                            p_next_node->p_s_father = p_cur_node;
//                            next_step = std::min((int)map_pose[I_STEP] + 1, half_sub_step);
//                            //if(next_step > p_next_node->step)
//                            {
//                                p_next_node->step = next_step;
//                                next_map_pose[I_STEP] = next_step;
//                                qu.push(next_map_pose);
//                            }
//                        }
//                        break;
//                    case 1:
//                        if(p_next_node->p_r_father == nullptr)
//                        {
//                            p_next_node->p_r_father = p_cur_node;
//                            next_step = 1;
//                            //if(next_step > p_next_node->step)
//                            {
//                                p_next_node->step = next_step;
//                                next_map_pose[I_STEP] = next_step;
//                                qu.push(next_map_pose);
//                            }
//                        }
//                        break;
//                    default:
//                        if(p_next_node->p_l_father == nullptr)
//                        {
//                            p_next_node->p_l_father = p_cur_node;
//                            next_step = 1;
//                            //if(next_step > p_next_node->step)
//                            {
//                                p_next_node->step = next_step;
//                                next_map_pose[I_STEP] = next_step;
//                                qu.push(next_map_pose);
//                            }
//                        }
//                        break;
//                    }
//                }
            }
        }
        //PRINT_DEBUG("");
    }

//    for(uint32_t ss = 1; ss < half_sub_step+1; ++ss)
//    {
//        for(uint32_t i = 0; i < 8; ++i)
//        {
//            for(uint32_t j = 0; j < map_w; ++j)
//            {
//                for(uint32_t k = 0; k < map_h; ++k)
//                {
//                    PRINT_DEBUG("{:d} {:d} {:d} {:d}", j, k, i, ss);
//                    for(int f = 0; f < 6; ++f)
//                    {
//                        Node* p_node = wave_4d_node[ss][i][j][k].p_fathers[f];
//                        if(p_node != nullptr)
//                        {
//                            PRINT_DEBUG("{:d}, {:d}, {:d}, {:d} [{:d}]<= {:d}, {:d}, {:d}, {:d} (%p)",
//                                        j, k, i, ss, f,
//                                        p_node->x, p_node->y, p_node->header, p_node->step, p_node);
//                        }
//                    }
//                }
//            }
//        }
//    }
//    *p_path = explore_point;
    PRINT_DEBUG("wave explore it = {}", it);
    //return true;
    if(is_find_path)
    {
        Node *p_start_node = &wave_4d_node[1][start_header_index][map_pose_start_x][map_pose_start_y];

//        int dx = -move_to_direct[goal_header_index][0][1] * (half_sub_step-1);
//        int dy = -move_to_direct[goal_header_index][0][2] * (half_sub_step-1);
        Node *p_goal_node = &wave_4d_node[half_sub_step][goal_header_index][map_pose_goal_x][map_pose_goal_y];
        class Graph
        {
        public:
            Graph(Graph *p_i_prev_graph = nullptr, Node *p_i_node=nullptr, float i_cost=0):
                p_prev_graph(p_i_prev_graph), p_node(p_i_node), cost(i_cost)
            {

            }
            Graph *p_prev_graph = nullptr;
            Node *p_node;
            float cost;
        };
        auto cmp = [&](const Graph* a, const Graph* b)
        {
            // 因为优先出列判定为!cmp，所以反向定义实现最小值优先
            return a->cost > b->cost;
        };

        std::priority_queue<Graph*, std::vector<Graph*>, decltype(cmp)> path_qu(cmp);
        Node *p_node = nullptr;
        Graph *p_graph = nullptr;
        //Graph graph_node = Graph(p_goal_node, 0);
//        std::map<Graph*, Graph> path_map;
        std::queue<Graph> graph_history;
        graph_history.push(Graph(nullptr, p_goal_node, 0));
        path_qu.push(&graph_history.back());

        const float map_cost_ratio = 0;
        while(!path_qu.empty())
        {
            p_graph = path_qu.top();
            path_qu.pop();
            p_node = p_graph->p_node;

//            PRINT_DEBUG("{:d}, {:d}, {:d}, {:d}", p_node->x, p_node->y, p_node->header, p_node->step);
            if(p_graph->p_node == p_start_node)
            {
                PRINT_DEBUG("djs find path!!!!!!!!!!!!!!!!!!!");
                while(p_graph != nullptr)
                {
                    PRINT_DEBUG("{:d}, {:d}, {:d}", p_graph->p_node->x, p_graph->p_node->y, p_graph->p_node->header);
                    Pose<float> pose;
                    mp_map->map_to_world((uint32_t)p_graph->p_node->x, (uint32_t)p_graph->p_node->y,
                                         &pose.position.x, &pose.position.y);
                    pose.heading_angle = p_graph->p_node->header * M_PI_4;
                    p_path->push_back(pose);
                    p_graph = p_graph->p_prev_graph;

                }
                break;
            }
            for(int i = 0; i < 6; ++i)
            {
                if(p_node->p_fathers[i] != nullptr)
                {
                    Node *p_next_node = p_node->p_fathers[i];
                    const float distance = hypot(p_node->x - p_next_node->x, p_node->y - p_next_node->y);
                    int8_t map_cost = mp_map->cost((uint32_t)p_next_node->x, (uint32_t)p_next_node->y);
                    float cost = p_graph->cost+ distance + (float)map_cost * map_cost_ratio;


                    if(p_next_node->path_cost == -1)
                    {
                        p_next_node->path_cost = cost;
                        graph_history.push(Graph(p_graph, p_next_node, cost));
                        path_qu.push(&graph_history.back());
//                        PRINT_DEBUG("{:d}, {:d}, {:d}, {:d} => {:d}, {:d}, {:d}, {:d}",
//                                    p_node->x, p_node->y, p_node->header, p_node->step,
//                                    p_next_node->x, p_next_node->y, p_next_node->header, p_next_node->step);
                    }
                    else
                    {
//                        if(cost < p_next_node->path_cost)
//                        {
//                            //PRINT_DEBUG("update cost");
//                            p_next_node->path_cost = cost;
//                            graph_history.push(Graph(p_graph, p_next_node, cost));
//                            path_qu.push(&graph_history.back());
////                            PRINT_DEBUG("{:d}, {:d}, {:d}, {:d} => {:d}, {:d}, {:d}, {:d}",
////                                        p_node->x, p_node->y, p_node->header, p_node->step,
////                                        p_next_node->x, p_next_node->y, p_next_node->header, p_next_node->step);
//                        }
                    }
                }
            }
            //PRINT_DEBUG("");
        }
    }


    //return true;
    return is_find_path;
}


bool RStarPlanner::directed_astar(const Pose<float> &start_pose, const Pose<float> &goal_pose, std::vector<Pose<float> > *p_path)
{
    //RECORD_TIME();
    class Node
    {
    public:
        Node(int mcost=-1):cost(mcost), step(0), is_closed(false),
            p_prev_node(nullptr){};
    public:
        int cost = -1;
        int step = 0;
        int x = 0;
        int y = 0;
        int header = 0;
        bool is_closed = false;
        //        bool is_visited = false;
        //        float path_cost = -1;
        Node* p_prev_node;
    };
    //wave_front_explore(start_pose, goal_pose);
    m_fmm_planner.fmm_explore(mp_map, start_pose, goal_pose);
    m_fmm_costmap = m_fmm_planner.costmap();

    typedef std::array<uint32_t, 5> Cell;
    const uint32_t I_X = 0;
    const uint32_t I_Y = 1;
    const uint32_t I_HEADING = 2;
    const uint32_t I_STEP = 3;
    const uint32_t I_COST = 4;
    const int8_t obstacle_value = mp_map->obstacles_cost()-1;
    uint32_t map_w = mp_map->size_in_cells_x();
    uint32_t map_h = mp_map->size_in_cells_y();
    const float r = 0.64 / tan(degree_to_radian(20));
    const int sub_step = ceil(r * tan(M_PI_4 * 0.5)*2 / mp_map->resolution())+1;
    //const int half_sub_step = std::max(1, (int)ceil(r * tan(M_PI_4 * 0.5) / mp_map->resolution()));
    const int half_sub_step = std::max(1, (int)round(sub_step * 0.5));
    PRINT_DEBUG("sub step = {:d}, half_sub_step = {:d}, sub_step * 0.5 = {}", sub_step, half_sub_step, (sub_step * 0.5));

    std::vector<std::vector<std::vector<std::vector<Node>>>> wave_4d_node;
    {
        //RECORD_TIME("allocate memory");
        wave_4d_node.resize(half_sub_step+1);
        for(uint32_t ss = 0; ss < half_sub_step+1; ++ss)
        {
            wave_4d_node[ss].resize(8);
            for(uint32_t i = 0; i < 8; ++i)
            {
                wave_4d_node[ss][i].resize(map_w);
                for(uint32_t j = 0; j < map_w; ++j)
                {
                    wave_4d_node[ss][i][j].resize(map_h);
                }
            }
        }

    }
    uint32_t map_pose_x;
    uint32_t map_pose_y;

    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_x, &map_pose_y);
    const uint32_t map_pose_start_x = map_pose_x;
    const uint32_t map_pose_start_y = map_pose_y;
    const uint32_t start_header_index = ((int)round(constraint_angle_r(start_pose.heading_angle) / M_PI_4)) % 8;
    //wave_3d_node[start_header_index][map_pose_start_x][map_pose_start_y].cost = 1;
    wave_4d_node[1][start_header_index][map_pose_start_x][map_pose_start_y].cost = 1;
    mp_map->world_to_map(goal_pose.position.x, goal_pose.position.y, &map_pose_x, &map_pose_y);
    const uint32_t map_pose_goal_x = map_pose_x;
    const uint32_t map_pose_goal_y = map_pose_y;
    uint32_t goal_header_index = ((int)round(constraint_angle_r(goal_pose.heading_angle) / M_PI_4)) % 8;


     /*
     *  3   2   1
     *  4   *   0
     *  5   6   7
     *  dx, dy, cost
     */

        static int move_to_direct[8][3][3] =
        {
            {{1, 0, 2}, {1, -1, 3}, {1,1, 3}},
            {{1, 1, 3}, {1, 0, 2}, {0, 1, 2}},
            {{0, 1, 2}, {1, 1, 3}, {-1, 1, 3}},
            {{-1, 1, 3}, {0, 1, 2}, {-1, 0, 2}},
            {{-1, 0, 2}, {-1, 1, 3}, {-1, -1, 3}},
            {{-1, -1, 3}, {-1, 0, 2}, {0, -1, 2}},
            {{0, -1, 2}, {-1,-1, 3}, {1, -1, 3}},
            {{1, -1, 3}, {0,-1, 2}, {1, 0, 2}},
        };
    static uint32_t move_direct_index[8][3] =
    {
        {0, 7, 1},
        {1, 0, 2},
        {2, 1, 3},
        {3, 2, 4},
        {4, 3, 5},
        {5, 4, 6},
        {6, 5, 7},
        {7, 6, 0},
    };

    bool is_find_path = false;
    Cell map_pose;
    uint32_t it = 0;
    p_path->clear();
    auto cmp = [&](const Cell& a, const Cell& b)
    {
        // 因为优先出列判定为!cmp，所以反向定义实现最小值优先
        return a[I_COST] > b[I_COST];
    };
    const uint32_t map_goal_preorder_x = round(map_pose_goal_x - half_sub_step * cos(goal_header_index * degree_to_radian(45)));
    const uint32_t map_goal_preorder_y = round(map_pose_goal_y - half_sub_step * sin(goal_header_index * degree_to_radian(45)));
    std::priority_queue<Cell, std::vector<Cell>, decltype(cmp)> qu(cmp);
    qu.push(Cell{map_pose_start_x, map_pose_start_y, start_header_index, 0, 1});
    //Node *p_cur_node = &wave_3d_node[start_header_index][map_pose_start_x][map_pose_start_y];
    Node *p_cur_node = &wave_4d_node[1][start_header_index][map_pose_start_x][map_pose_start_y];
    p_cur_node->x = map_pose_start_x;
    p_cur_node->y = map_pose_start_y;
    p_cur_node->header = start_header_index;
    p_cur_node->cost = 1;
    Node *p_next_node = nullptr;
    while(!qu.empty())
    {
        if(it % 100 == 0)
        {
            if(m_flag_stop)
            {
                m_flag_stop = false;
                PRINT_DEBUG("recv stop flag, total it = {}", it);
                return false;
            }
        }
        map_pose = qu.top();
        qu.pop();
        //explore wave
        //p_cur_node = &wave_3d_node[map_pose[I_HEADING]][map_pose[I_X]][map_pose[I_Y]];
        p_cur_node = &wave_4d_node[map_pose[I_STEP]][map_pose[I_HEADING]][map_pose[I_X]][map_pose[I_Y]];
        if(p_cur_node->is_closed)
        {
            continue;
        }
        else
        {
            p_cur_node->is_closed = true;
        }

        if(map_pose[I_X] == map_pose_goal_x && map_pose[I_Y] == map_pose_goal_y &&
            goal_header_index == map_pose[I_HEADING] && map_pose[I_STEP] == half_sub_step)
        {
            is_find_path = true;
            PRINT_DEBUG("find path!!!");
            break;
        }

        for(int i = 0; i < 3; ++i)
        {
            if(i > 0 && map_pose[I_STEP] < half_sub_step)
            {
                break;
            }
            ++it;
            Cell next_map_pose;
            next_map_pose[I_X] = map_pose[I_X] + move_to_direct[map_pose[I_HEADING]][i][0];
            next_map_pose[I_Y] = map_pose[I_Y] + move_to_direct[map_pose[I_HEADING]][i][1];
            next_map_pose[I_HEADING] = move_direct_index[map_pose[I_HEADING]][i];
            if(next_map_pose[I_X] < map_w && next_map_pose[I_Y] < map_h)
            {
                next_map_pose[I_STEP] = 1;
                if(i == 0)
                {
                    next_map_pose[I_STEP] = std::min((int)map_pose[I_STEP] + 1, half_sub_step);
                }
                //p_next_node = &wave_3d_node[next_map_pose[I_HEADING]][next_map_pose[I_X]][next_map_pose[I_Y]];
                p_next_node = &wave_4d_node[next_map_pose[I_STEP]][next_map_pose[I_HEADING]][next_map_pose[I_X]][next_map_pose[I_Y]];
                int map_cost = (int)mp_map->cost((uint32_t)next_map_pose[I_X], (uint32_t)next_map_pose[I_Y]);
                int cost = p_cur_node->cost + move_to_direct[map_pose[I_HEADING]][i][2] * 10 + map_cost * 0.00
                           + ((i+1) / 2) * 0;
                if(p_next_node->cost == -1)
                {
                    if(map_cost < obstacle_value)
                    {
                        //next_map_pose[I_COST] = cost + m_wave_explore_costmap[next_map_pose[I_X]][next_map_pose[I_Y]];
                        next_map_pose[I_COST] = cost + round(m_fmm_costmap[next_map_pose[I_X]][next_map_pose[I_Y]] * 3);
                        p_next_node->cost = cost;
                        p_next_node->p_prev_node = p_cur_node;
                        p_next_node->step = next_map_pose[I_STEP];
                        p_next_node->x = next_map_pose[I_X];
                        p_next_node->y = next_map_pose[I_Y];
                        p_next_node->header = next_map_pose[I_HEADING];
                        qu.push(next_map_pose);
                    }
                }
                else
                {
                    if(cost < p_next_node->cost)
                    {
                        p_next_node->cost = cost;
                        p_next_node->p_prev_node = p_cur_node;
                        p_next_node->step = next_map_pose[I_STEP];
                        p_next_node->x = next_map_pose[I_X];
                        p_next_node->y = next_map_pose[I_Y];
                        p_next_node->header = next_map_pose[I_HEADING];
                        p_next_node->is_closed = false;
                        qu.push(next_map_pose);
                    }
                }
            }
        }
        //PRINT_DEBUG("");
    }
    if(is_find_path)
    {
        while(p_cur_node != nullptr)
        {
            Pose<float> pose;
            //mp_map->map_to_world((uint32_t)p_cur_node->x, (uint32_t)p_cur_node->y, &pose.position.x, &pose.position.y);
            pose.position.x = p_cur_node->x;
            pose.position.y = p_cur_node->y;
            pose.heading_angle = p_cur_node->header * degree_to_radian(45);
            p_path->emplace_back(pose);
            p_cur_node = p_cur_node->p_prev_node;
        }

        p_path->pop_back();
        std::reverse(p_path->begin(), p_path->end());
        const int path_size = (*p_path).size();
        for(int i = 0; i < path_size; ++i)
        {
            mp_map->map_to_world((*p_path)[i].position.x, (*p_path)[i].position.y,
                                 &(*p_path)[i].position.x, &(*p_path)[i].position.y);
        }

        VectorX2<float> start_pose_error = start_pose.position - (*p_path)[0].position;
        VectorX2<float> goal_pose_error = goal_pose.position - (*p_path)[path_size-1].position;

        for(int i = 0; i < path_size; ++i)
        {
            if(i < path_size/2)
            {
                (*p_path)[i].position += start_pose_error * (1.0 * (path_size/2 - i) / (path_size/2));
            }
            else
            {
                (*p_path)[i].position += goal_pose_error * (1.0 * (i - (path_size/2)) / (path_size - 1 - path_size/2));
            }
        }
        p_path->back() = goal_pose;
        //p_path->push_back(goal_pose);

//smooth path
#if 0
        //{
        for(int i = 0; i < path_size; ++i)
        {
            mp_map->world_to_map((*p_path)[i].position.x, (*p_path)[i].position.y,
                                 &(*p_path)[i].position.x, &(*p_path)[i].position.y);
        }

        std::vector<Pose<float>> smooth_path;
        smooth_path = *p_path;
        bool is_enable_smooth = true;
//        if(is_enable_smooth)
//        {
//            mp_path_smooth->update_map(mp_map);
//            mp_path_smooth->smoothPath(*p_path);
//            smooth_path = mp_path_smooth->getSmoothedPath();
//        }

        for(int i = 0; i < smooth_path.size(); ++i)
        {
            mp_map->map_to_world((*p_path)[i].position.x, (*p_path)[i].position.y,
                                 &(*p_path)[i].position.x, &(*p_path)[i].position.y);

            mp_map->map_to_world(smooth_path[i].position.x, smooth_path[i].position.y,
                                 &smooth_path[i].position.x, &smooth_path[i].position.y);
        }
        *p_path = smooth_path;
#endif
        //p_path->insert(p_path->end(), smooth_path.begin(), smooth_path.end());
    }

    PRINT_DEBUG("total it = {}, result = {}", it, is_find_path);
    return is_find_path;
}

bool RStarPlanner::directed_astar2(const Pose<float> &start_pose, const Pose<float> &goal_pose, std::vector<Pose<float> > *p_path)
{
    //RECORD_TIME();
    class Node
    {
    public:
        Node(int mcost=-1):cost(mcost), step(0), is_closed(false),
            p_prev_node(nullptr){}
    public:
        int cost = -1;
        int step = 0;
        int x = 0;
        int y = 0;
        int header = 0;
        bool is_closed = false;
        Node* p_prev_node;
    };

    class Cell
    {
    public:
        Cell() {}
        Cell(uint32_t ix, uint32_t iy, uint32_t iheading_index, uint32_t istep,
             float icost, uint32_t iswitch_heading_index, uint32_t iswitch_step):
            x(ix),
            y(iy),
            heading_index(iheading_index),
            step(istep),
            cost(icost),
            switch_heading_index(iswitch_heading_index),
            switch_step(iswitch_step) {}
    public:
        uint32_t x;
        uint32_t y;
        uint32_t heading_index;
        uint32_t step;
        float cost;
        //表示转向这一个heading的heading
        uint32_t switch_heading_index;
        //表示转向这一个heading的step
        uint32_t switch_step;
    };
    auto cmp = [&](const Cell& a, const Cell& b)
    {
        // 因为优先出列判定为!cmp，所以反向定义实现最小值优先
        return a.cost + m_fmm_costmap[a.x][a.y] * 5.0f >
               b.cost + m_fmm_costmap[b.x][b.y] * 5.0f;
    };

    //wave_front_explore(start_pose, goal_pose);
    bool is_find_path = false;
    Cell map_pose;
    uint32_t it = 0;

    uint32_t map_pose_x;
    uint32_t map_pose_y;

    const int8_t obstacle_value = mp_map->obstacles_cost()-1;
    uint32_t map_w = mp_map->size_in_cells_x();
    uint32_t map_h = mp_map->size_in_cells_y();
    const float r = 0.64 / tan(degree_to_radian(20));
    const int sub_step = ceil(r * tan(M_PI_4 * 0.5)*2 / mp_map->resolution());
    //const int half_sub_step = std::max(1, (int)ceil(r * tan(M_PI_4 * 0.5) / mp_map->resolution())+ 1);
    const int half_sub_step = std::max(1, (int)round(sub_step * 0.5));
    PRINT_DEBUG("sub step = {:d}, half_sub_step = {:d}, sub_step * 0.5 = {}", sub_step, half_sub_step, (sub_step * 0.5));
    p_path->clear();
    m_fmm_planner.fmm_explore(mp_map, start_pose, goal_pose);
    m_fmm_costmap = m_fmm_planner.costmap();
    std::vector<std::vector<std::vector<std::vector<Node>>>> wave_4d_node;
    {
        //RECORD_TIME("allocate memory");
        //wave_4d_node.resize(half_sub_step+1);
        //for(uint32_t ss = 0; ss < half_sub_step+1; ++ss)
        wave_4d_node.resize(sub_step+1);
        for(uint32_t ss = 0; ss < sub_step+1; ++ss)
        {
            wave_4d_node[ss].resize(8);
            for(uint32_t i = 0; i < 8; ++i)
            {
                wave_4d_node[ss][i].resize(map_w);
                for(uint32_t j = 0; j < map_w; ++j)
                {
                    wave_4d_node[ss][i][j].resize(map_h);
                }
            }
        }

    }

    mp_map->world_to_map(start_pose.position.x, start_pose.position.y, &map_pose_x, &map_pose_y);
    const uint32_t map_pose_start_x = map_pose_x;
    const uint32_t map_pose_start_y = map_pose_y;
    const uint32_t start_header_index = ((int)round(constraint_angle_r(start_pose.heading_angle) / M_PI_4)) % 8;
    //wave_3d_node[start_header_index][map_pose_start_x][map_pose_start_y].cost = 1;
    wave_4d_node[1][start_header_index][map_pose_start_x][map_pose_start_y].cost = 1;
    mp_map->world_to_map(goal_pose.position.x, goal_pose.position.y, &map_pose_x, &map_pose_y);
    const uint32_t map_pose_goal_x = map_pose_x;
    const uint32_t map_pose_goal_y = map_pose_y;
    uint32_t goal_header_index = ((int)round(constraint_angle_r(goal_pose.heading_angle) / M_PI_4)) % 8;


        /*
     *  3   2   1
     *  4   *   0
     *  5   6   7
     *  dx, dy, cost
     */

    static int move_to_direct[8][3][3] =
    {
        {{1, 0, 2}, {1, -1, 3}, {1,1, 3}},
        {{1, 1, 3}, {1, 0, 2}, {0, 1, 2}},
        {{0, 1, 2}, {1, 1, 3}, {-1, 1, 3}},
        {{-1, 1, 3}, {0, 1, 2}, {-1, 0, 2}},
        {{-1, 0, 2}, {-1, 1, 3}, {-1, -1, 3}},
        {{-1, -1, 3}, {-1, 0, 2}, {0, -1, 2}},
        {{0, -1, 2}, {-1,-1, 3}, {1, -1, 3}},
        {{1, -1, 3}, {0,-1, 2}, {1, 0, 2}},
     };
    static uint32_t move_direct_index[8][3] =
    {
        {0, 7, 1},
        {1, 0, 2},
        {2, 1, 3},
        {3, 2, 4},
        {4, 3, 5},
        {5, 4, 6},
        {6, 5, 7},
        {7, 6, 0},
    };


    const uint32_t map_goal_preorder_x = round(map_pose_goal_x - half_sub_step * cos(goal_header_index * degree_to_radian(45)));
    const uint32_t map_goal_preorder_y = round(map_pose_goal_y - half_sub_step * sin(goal_header_index * degree_to_radian(45)));
    std::priority_queue<Cell, std::vector<Cell>, decltype(cmp)> qu(cmp);
    qu.push(Cell{map_pose_start_x, map_pose_start_y, start_header_index, std::max(half_sub_step-1, 1), 1, start_header_index, 1});
    //Node *p_cur_node = &wave_3d_node[start_header_index][map_pose_start_x][map_pose_start_y];
    Node *p_cur_node = &wave_4d_node[1][start_header_index][map_pose_start_x][map_pose_start_y];
    p_cur_node->x = map_pose_start_x;
    p_cur_node->y = map_pose_start_y;
    p_cur_node->header = start_header_index;
    p_cur_node->cost = 1;
    Node *p_next_node = nullptr;
    while(!qu.empty())
    {
//        if(it % 100 == 0)
//        {
//            if(m_flag_stop)
//            {
//                m_flag_stop = false;
//                PRINT_DEBUG("recv stop flag, total it = {}", it);
//                return false;
//            }
//        }
        if(it > 100)
        {
            it = 0;
            if(m_flag_stop)
            {
                m_flag_stop = false;
                PRINT_DEBUG("planner stop");
                return false;
            }
        }
        map_pose = qu.top();
        qu.pop();
        //explore wave
        //p_cur_node = &wave_3d_node[map_pose[I_HEADING]][map_pose[I_X]][map_pose[I_Y]];
        p_cur_node = &wave_4d_node[map_pose.step][map_pose.heading_index][map_pose.x][map_pose.y];
        if(p_cur_node->is_closed)
        {
            continue;
        }
        else
        {
            p_cur_node->is_closed = true;
        }

        if(map_pose.x == map_pose_goal_x && map_pose.y == map_pose_goal_y &&
            goal_header_index == map_pose.heading_index && map_pose.step >= half_sub_step)
        {
            is_find_path = true;
            PRINT_DEBUG("find path!!!");
            break;
        }

        for(int i = 0; i < 3; ++i)
        {
//            //TODO: modify
//            if(i > 0 && map_pose.step < half_sub_step)
//            {
//                break;
//            }
//            //END of TODO

            if(i > 0 && map_pose.switch_heading_index != move_direct_index[map_pose.heading_index][i]
                     && map_pose.step != sub_step)
            {
                continue;
            }

            if(i > 0 && map_pose.step < half_sub_step)
            {
                if(map_pose.switch_heading_index == move_direct_index[map_pose.heading_index][i] &&
                    map_pose.switch_step >= half_sub_step)
                {
                    //skip
                }
                else
                {
                    continue;
                }
            }

            ++it;
            Cell next_map_pose;
            next_map_pose.x = map_pose.x + move_to_direct[map_pose.heading_index][i][0];
            next_map_pose.y = map_pose.y + move_to_direct[map_pose.heading_index][i][1];
            next_map_pose.heading_index = move_direct_index[map_pose.heading_index][i];
            if(i != 0)
            {
                next_map_pose.switch_heading_index = map_pose.heading_index;
                next_map_pose.switch_step = map_pose.step;
            }
            else
            {
                next_map_pose.switch_heading_index = map_pose.switch_heading_index;
                next_map_pose.switch_step = map_pose.switch_step;
            }

            if(next_map_pose.x < map_w && next_map_pose.y < map_h)
            {
                if(i == 0)
                {
                    next_map_pose.step = std::min((int)map_pose.step + 1, sub_step);
                }
                else
                {
                    next_map_pose.step = 1;
                }

                p_next_node = &wave_4d_node[next_map_pose.step][next_map_pose.heading_index][next_map_pose.x][next_map_pose.y];
                int map_cost = (int)mp_map->cost(next_map_pose.x, next_map_pose.y);
                int cost = p_cur_node->cost + 1.5f * (float)move_to_direct[map_pose.heading_index][i][2] + map_cost * 0.07f
                           + ((i+1) / 2) * 2.0f;
                if(p_next_node->cost == -1)
                {
                    if(map_cost < obstacle_value)
                    {
                        //next_map_pose[I_COST] = cost + m_wave_explore_costmap[next_map_pose[I_X]][next_map_pose[I_Y]];
                        //next_map_pose.cost = cost + round(m_fmm_costmap[next_map_pose.x][next_map_pose.y] * 3.0f);
                        next_map_pose.cost = cost;
                        p_next_node->cost = cost;
                        p_next_node->p_prev_node = p_cur_node;
                        p_next_node->step = next_map_pose.step;
                        p_next_node->x = next_map_pose.x;
                        p_next_node->y = next_map_pose.y;
                        p_next_node->header = next_map_pose.heading_index;
                        qu.push(next_map_pose);
                    }
                }
                else
                {
                    if(cost < p_next_node->cost)
                    {
                        p_next_node->cost = cost;
                        p_next_node->p_prev_node = p_cur_node;
                        p_next_node->step = next_map_pose.step;
                        p_next_node->x = next_map_pose.x;
                        p_next_node->y = next_map_pose.y;
                        p_next_node->header = next_map_pose.heading_index;
                        p_next_node->is_closed = false;
                        qu.push(next_map_pose);
                    }
                }
            }
        }
        //PRINT_DEBUG("");
    }
    if(is_find_path)
    {
        while(p_cur_node != nullptr)
        {
            Pose<float> pose;
            //mp_map->map_to_world((uint32_t)p_cur_node->x, (uint32_t)p_cur_node->y, &pose.position.x, &pose.position.y);
            pose.position.x = p_cur_node->x;
            pose.position.y = p_cur_node->y;
            pose.heading_angle = p_cur_node->header * degree_to_radian(45);
            p_path->emplace_back(pose);
            p_cur_node = p_cur_node->p_prev_node;
        }

        p_path->pop_back();
        std::reverse(p_path->begin(), p_path->end());
        const int path_size = (*p_path).size();
        for(int i = 0; i < path_size; ++i)
        {
            mp_map->map_to_world((*p_path)[i].position.x, (*p_path)[i].position.y,
                                 &(*p_path)[i].position.x, &(*p_path)[i].position.y);
        }

        VectorX2<float> start_pose_error = start_pose.position - (*p_path)[0].position;
        VectorX2<float> goal_pose_error = goal_pose.position - (*p_path)[path_size-1].position;

        for(int i = 0; i < path_size; ++i)
        {
            if(i < path_size/2)
            {
                (*p_path)[i].position += start_pose_error * (1.0 * (path_size/2 - i) / (path_size/2));
            }
            else
            {
                (*p_path)[i].position += goal_pose_error * (1.0 * (i - (path_size/2)) / (path_size - 1 - path_size/2));
            }
        }
        p_path->back() = goal_pose;
        //p_path->push_back(goal_pose);

//smooth path
#if 0
        //{
        for(int i = 0; i < path_size; ++i)
        {
            mp_map->world_to_map((*p_path)[i].position.x, (*p_path)[i].position.y,
                                 &(*p_path)[i].position.x, &(*p_path)[i].position.y);
        }

        std::vector<Pose<float>> smooth_path;
        smooth_path = *p_path;
        bool is_enable_smooth = true;
        if(is_enable_smooth)
        {
            mp_path_smooth->update_map(mp_map);
            mp_path_smooth->smoothPath(*p_path);
            smooth_path = mp_path_smooth->getSmoothedPath();
            for(int i = 0; i < smooth_path.size(); ++i)
            {
                mp_map->map_to_world((*p_path)[i].position.x, (*p_path)[i].position.y,
                                     &(*p_path)[i].position.x, &(*p_path)[i].position.y);

                mp_map->map_to_world(smooth_path[i].position.x, smooth_path[i].position.y,
                                     &smooth_path[i].position.x, &smooth_path[i].position.y);
            }
            *p_path = smooth_path;
        }


        p_path->front() = start_pose;
        p_path->back() = goal_pose;
#endif
        //p_path->insert(p_path->end(), smooth_path.begin(), smooth_path.end());
    }

    PRINT_DEBUG("total it = {}, result = {}", it, is_find_path);
    return is_find_path;
}

std::vector<Pose<float> > RStarPlanner::smooth(std::vector<Pose<float>> *p_path, float weight_data, float weight_smooth, float tolerance)
{
    std::vector<Pose<float>> newpath = *p_path;
    float change = tolerance;
    while(change >= tolerance)
    {
        change = 0;
        for(int i = 1; i < p_path->size()-1; ++i)
        {
            float d1 = weight_data * ((*p_path)[i].position.x - newpath[i].position.x);
            float d2 = weight_smooth * (newpath[i-1].position.x + newpath[i+1].position.x - 2 * newpath[i].position.x);
            change += fabs(d1 + d2);
            newpath[i].position.x += (d1 + d2);

            d1 = weight_data * ((*p_path)[i].position.y - newpath[i].position.y);
            d2 = weight_smooth * (newpath[i-1].position.y + newpath[i+1].position.y - 2 * newpath[i].position.y);
            change += fabs(d1 + d2);
            newpath[i].position.y += (d1 + d2);
        }
    }
    return newpath;
}
#if 0
void RStarPlanner::inflate_path(const std::vector<Pose<float>> &original_path)
{
    RECORD_TIME();
    std::vector<Pose<float> > path = interpolate_path(original_path, mp_map->resolution());
    const static int pos_array[8][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1},  {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
    const uint32_t array_size = sizeof(pos_array) / sizeof(pos_array[0]);
    std::vector<VectorX2<uint32_t>> map_path;
    std::queue<VectorX3<uint32_t>> qu;
    uint32_t path_size = path.size();
    const int8_t obstacle_value = mp_map->obstacles_cost()-1;
    uint32_t map_w = mp_map->size_in_cells_x();
    uint32_t map_h = mp_map->size_in_cells_y();
    std::vector<std::vector<int>> empty_map;
    empty_map.resize(map_w);
    for(uint32_t i = 0; i < map_w; ++i)
    {
        empty_map[i].assign(map_h, 255);
    }

    map_path.resize(path_size);
    for(uint32_t i = 0; i < path_size; ++i)
    {
        if(mp_map->world_to_map(path[i].position.x, path[i].position.y, &map_path[i].x, &map_path[i].y))
        {
            qu.push(VectorX3<uint32_t>(map_path[i].x, map_path[i].y, 0));
            empty_map[map_path[i].x][map_path[i].y] = 0;
            //PRINT_INFO("empty_map[{:d}][{:d}] = {:d}", map_path[i].x, map_path[i].y, empty_map[map_path[i].x][map_path[i].y]);
            continue;
        }
        else
        {
            PRINT_ERROR("path world pose to map error");
            return;
        }
    }

    while(!qu.empty())
    {

        VectorX3<uint32_t> map_pose = qu.front();
        qu.pop();

        for(int i = 0; i < array_size; ++i)
        {
            //++it;
            VectorX3<uint32_t> next_map_pose;
            next_map_pose.x = map_pose.x + pos_array[i][0];
            next_map_pose.y = map_pose.y + pos_array[i][1];
            next_map_pose.z = map_pose.z + 1;
            if(next_map_pose.x < map_w && next_map_pose.y < map_h)
            {
                if(empty_map[next_map_pose.x][next_map_pose.y] == 255)
                {
                    uint8_t map_cost = mp_map->cost((uint32_t)next_map_pose.x, (uint32_t)next_map_pose.y);
                    const float drop_speed = 0.05;
                    int cost = obstacle_value - exp(-1.0 * drop_speed * next_map_pose.z) * obstacle_value;
                    if(cost < empty_map[next_map_pose.x][next_map_pose.y] && map_cost < obstacle_value)
                    {
                        empty_map[next_map_pose.x][next_map_pose.y] = cost;
                        qu.push(next_map_pose);
                        PRINT_DEBUG("empty_map[{:d}][{:d}] = {:d} - {:d}", next_map_pose.x, next_map_pose.y, next_map_pose.z,
                                    cost);
                    }
                }
            }
        }
    }

#define DEBUG_WRITE_COST 1
#ifdef DEBUG_WRITE_COST
    uint32_t size = map_w * map_h;
    uint8_t *map_data = (uint8_t*)malloc(size);
    for(int i = 0; i < size; ++i)
    {
        uint32_t y = i / map_w;
        uint32_t x = i - y * map_w;
        int value = empty_map[x][y];
        map_data[i] = 255 - value;
    }
    write_pnm_image(map_data, map_w, map_h, "/home/zhou/local_map.pgm", pnm_t::P5);
    free(map_data);
#endif
}
#endif

}
}
