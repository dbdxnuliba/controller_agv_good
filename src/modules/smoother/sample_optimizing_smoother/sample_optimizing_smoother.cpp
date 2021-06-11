#include "sample_optimizing_smoother.h"
#include "path_optimizer.hpp"


namespace bz_robot
{

class SampleOptimizingSmootherParams
{
public:
    std::shared_ptr<PathOptimizationNS::Configs> p_smoother_configs;
    PathData smoothed_path;
};


SampleOptimizingSmoother::SampleOptimizingSmoother()
{
    mp_params = std::make_shared<SampleOptimizingSmootherParams>();
    mp_params->p_smoother_configs = std::make_shared<PathOptimizationNS::Configs>();
    mp_params->p_smoother_configs->update_config();
}

bool SampleOptimizingSmoother::import_config(const char *config_file)
{
    try
    {
        PRINT_INFO("read config: {}", config_file);
        std::ifstream i(config_file);
        if(i)
        {
            nlohmann::json j;
            i >> j;
            mp_params->p_smoother_configs->FLAGS_car_width = j["FLAGS_car_width"];
            mp_params->p_smoother_configs->FLAGS_car_length = j["FLAGS_car_length"];
            mp_params->p_smoother_configs->FLAGS_safety_margin = j["FLAGS_safety_margin"];
            mp_params->p_smoother_configs->FLAGS_wheel_base = j["FLAGS_wheel_base"];
            mp_params->p_smoother_configs->FLAGS_rear_axle_to_center = j["FLAGS_rear_axle_to_center"];
            mp_params->p_smoother_configs->FLAGS_max_steering_angle = j["FLAGS_max_steering_angle"];
            mp_params->p_smoother_configs->FLAGS_max_steering_angle = degree_to_radian(mp_params->p_smoother_configs->FLAGS_max_steering_angle);
            mp_params->p_smoother_configs->FLAGS_mu = j["FLAGS_mu"];
            mp_params->p_smoother_configs->FLAGS_max_curvature_rate = j["FLAGS_max_curvature_rate"];
            mp_params->p_smoother_configs->FLAGS_smoothing_method = j["FLAGS_smoothing_method"];
            mp_params->p_smoother_configs->FLAGS_tension_solver = j["FLAGS_tension_solver"];
            mp_params->p_smoother_configs->FLAGS_enable_searching = j["FLAGS_enable_searching"];
            mp_params->p_smoother_configs->FLAGS_search_lateral_range = j["FLAGS_search_lateral_range"];
            mp_params->p_smoother_configs->FLAGS_search_longitudial_spacing = j["FLAGS_search_longitudial_spacing"];
            mp_params->p_smoother_configs->FLAGS_search_lateral_spacing = j["FLAGS_search_lateral_spacing"];
            mp_params->p_smoother_configs->FLAGS_frenet_angle_diff_weight = j["FLAGS_frenet_angle_diff_weight"];
            mp_params->p_smoother_configs->FLAGS_frenet_angle_diff_diff_weight = j["FLAGS_frenet_angle_diff_diff_weight"];
            mp_params->p_smoother_configs->FLAGS_frenet_deviation_weight = j["FLAGS_frenet_deviation_weight"];
            mp_params->p_smoother_configs->FLAGS_cartesian_curvature_weight = j["FLAGS_cartesian_curvature_weight"];
            mp_params->p_smoother_configs->FLAGS_cartesian_curvature_rate_weight = j["FLAGS_cartesian_curvature_rate_weight"];
            mp_params->p_smoother_configs->FLAGS_cartesian_deviation_weight = j["FLAGS_cartesian_deviation_weight"];
            mp_params->p_smoother_configs->FLAGS_tension_2_deviation_weight = j["FLAGS_tension_2_deviation_weight"];
            mp_params->p_smoother_configs->FLAGS_tension_2_curvature_weight = j["FLAGS_tension_2_curvature_weight"];
            mp_params->p_smoother_configs->FLAGS_tension_2_curvature_rate_weight = j["FLAGS_tension_2_curvature_rate_weight"];
            mp_params->p_smoother_configs->FLAGS_enable_simple_boundary_decision = j["FLAGS_enable_simple_boundary_decision"];
            mp_params->p_smoother_configs->FLAGS_search_obstacle_cost = j["FLAGS_search_obstacle_cost"];
            mp_params->p_smoother_configs->FLAGS_search_deviation_cost = j["FLAGS_search_deviation_cost"];
            mp_params->p_smoother_configs->FLAGS_optimization_method = j["FLAGS_optimization_method"];
            mp_params->p_smoother_configs->FLAGS_K_curvature_weight = j["FLAGS_K_curvature_weight"];
            mp_params->p_smoother_configs->FLAGS_K_curvature_rate_weight = j["FLAGS_K_curvature_rate_weight"];
            mp_params->p_smoother_configs->FLAGS_K_deviation_weight = j["FLAGS_K_deviation_weight"];
            mp_params->p_smoother_configs->FLAGS_KP_curvature_weight = j["FLAGS_KP_curvature_weight"];
            mp_params->p_smoother_configs->FLAGS_KP_curvature_rate_weight = j["FLAGS_KP_curvature_rate_weight"];
            mp_params->p_smoother_configs->FLAGS_KP_deviation_weight = j["FLAGS_KP_deviation_weight"];
            mp_params->p_smoother_configs->FLAGS_KP_slack_weight = j["FLAGS_KP_slack_weight"];
            mp_params->p_smoother_configs->FLAGS_expected_safety_margin = j["FLAGS_expected_safety_margin"];
            mp_params->p_smoother_configs->FLAGS_constraint_end_heading = j["FLAGS_constraint_end_heading"];
            mp_params->p_smoother_configs->FLAGS_enable_exact_position = j["FLAGS_enable_exact_position"];
            mp_params->p_smoother_configs->FLAGS_enable_raw_output = j["FLAGS_enable_raw_output"];
            mp_params->p_smoother_configs->FLAGS_output_spacing = j["FLAGS_output_spacing"];
            mp_params->p_smoother_configs->FLAGS_enable_collision_check = j["FLAGS_enable_collision_check"];
            mp_params->p_smoother_configs->FLAGS_enable_dynamic_segmentation = j["FLAGS_enable_dynamic_segmentation"];

            mp_params->p_smoother_configs->update_config();
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

bool SampleOptimizingSmoother::smooth(std::shared_ptr<MapBase> p_map, const PathData &ref_path)
{
    bool result = false;
    std::vector<PathOptimizationNS::State> result_path;
    std::vector<PathOptimizationNS::State> reference_path;
    PathOptimizationNS::State state_start(ref_path[0].position.x, ref_path[0].position.y, ref_path[0].heading_angle);
    PathOptimizationNS::State state_goal(ref_path.back().position.x, ref_path.back().position.y, ref_path.back().heading_angle);
    PathOptimizationNS::PathOptimizer path_optimizer(state_start, state_goal, p_map, mp_params->p_smoother_configs);
    reference_path.resize(ref_path.size());
    for(int i = 0; i != reference_path.size(); ++i)
    {
        reference_path[i].x = ref_path[i].position.x;
        reference_path[i].y = ref_path[i].position.y;
        reference_path[i].z = ref_path[i].heading_angle;
    }
    if (path_optimizer.solve(reference_path, &result_path))
    {
        result = true;
        // Test solveWithoutSmoothing:
        //path_optimizer.solveWithoutSmoothing(result_path, &result_path);
        mp_params->smoothed_path.resize(result_path.size());
        for(int i = 0; i != result_path.size(); ++i)
        {
            mp_params->smoothed_path[i].position.x = result_path[i].x;
            mp_params->smoothed_path[i].position.y = result_path[i].y;
            mp_params->smoothed_path[i].heading_angle = result_path[i].z;
        }
    }
    else
    {
        mp_params->smoothed_path = ref_path;
    }
    return result;
}

PathData SampleOptimizingSmoother::smoothed_path()
{
    return mp_params->smoothed_path;
}

}
