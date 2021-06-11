#include "gradient_descent_smoother.h"
#include "gradient_descent_smoother/path_smoother.h"


namespace bz_robot
{

class GradientDescentSmootherParams
{
public:
    FLOAT_T w_alpha = 0.1;
    FLOAT_T w_obstacle = 0.1;
    FLOAT_T w_voronoi = 0.3;
    FLOAT_T w_curvature = 0.5;
    FLOAT_T w_smoothness = 0.5;

    FLOAT_T min_turn_radius = 2;
    FLOAT_T min_road_width = 1;
    uint32_t max_iterations = 500;
    std::shared_ptr<PathSmoother> p_smoother;
    PathData smoothed_path;
};


GradientDescentSmoother::GradientDescentSmoother()
{
    mp_params = std::make_shared<GradientDescentSmootherParams>();
    mp_params->p_smoother = std::make_shared<PathSmoother>();
}

bool GradientDescentSmoother::import_config(const char *config_file)
{
    try
    {
        PRINT_INFO("read config: {}", config_file);
        std::ifstream i(config_file);
        if(i)
        {
            nlohmann::json j;
            i >> j;
            mp_params->w_alpha = j["w_alpha"];
            mp_params->w_obstacle = j["w_obstacle"];
            mp_params->w_voronoi = j["w_voronoi"];
            mp_params->w_curvature = j["w_curvature"];
            mp_params->w_smoothness = j["w_smoothness"];
            mp_params->min_turn_radius = j["min_turn_radius"];
            mp_params->min_road_width = j["min_road_width"];
            mp_params->max_iterations = j["max_iterations"];

            mp_params->p_smoother->m_alpha = mp_params->w_alpha;
            mp_params->p_smoother->m_w_obstacle = mp_params->w_obstacle;
            mp_params->p_smoother->m_w_voronoi = mp_params->w_voronoi;
            mp_params->p_smoother->m_w_curvature = mp_params->w_curvature;
            mp_params->p_smoother->m_w_smoothness = mp_params->w_smoothness;
            mp_params->p_smoother->m_min_turn_radius = mp_params->min_turn_radius;
            mp_params->p_smoother->m_min_road_width = mp_params->min_road_width;
            mp_params->p_smoother->m_max_iterations = mp_params->max_iterations;

            return true;
        }
        else
        {
            PRINT_ERROR("can't read config files from: {}\n", config_file);
        }
    }
    catch(std::exception& e )
    {
        PRINT_ERROR("{}\n", e.what());
    }
    catch(...)
    {
        PRINT_ERROR("un expexted occured\n");
    }
    return false;
}

bool GradientDescentSmoother::smooth(std::shared_ptr<MapBase> p_map, const PathData &ref_path)
{
    mp_params->p_smoother->update_map(p_map);
    PathData map_ref_path = ref_path;
    for(int i = 0; i < map_ref_path.size(); ++i)
    {
        p_map->world_to_map(map_ref_path[i].position.x, map_ref_path[i].position.y,
                            &map_ref_path[i].position.x, &map_ref_path[i].position.y);
    }

    mp_params->p_smoother->smoothPath(map_ref_path);
    mp_params->smoothed_path = std::move(mp_params->p_smoother->getSmoothedPath());
    for(int i = 0; i < mp_params->smoothed_path.size(); ++i)
    {
        p_map->map_to_world(mp_params->smoothed_path[i].position.x, mp_params->smoothed_path[i].position.y,
                            &mp_params->smoothed_path[i].position.x, &mp_params->smoothed_path[i].position.y);
    }

    return true;
}

PathData GradientDescentSmoother::smoothed_path()
{
    return mp_params->smoothed_path;
}

	
}
