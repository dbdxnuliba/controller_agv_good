//
// Created by ljn on 20-4-12.
//

#pragma once

#include <string>
namespace bz_robot
{
namespace PathOptimizationNS
{
class Configs
{
public:
    Configs() {}
    void update_config() {
        FLAGS_circle_radius = sqrt(pow(FLAGS_car_length / 8, 2) + pow(FLAGS_car_width / 2, 2)) + FLAGS_safety_margin;
        FLAGS_d1 = -3.0 / 8.0 * FLAGS_car_length + FLAGS_rear_axle_to_center;
        FLAGS_d2 = -1.0 / 8.0 * FLAGS_car_length + FLAGS_rear_axle_to_center;
        FLAGS_d3 = 1.0 / 8.0 * FLAGS_car_length + FLAGS_rear_axle_to_center;
        FLAGS_d4 = 3.0 / 8.0 * FLAGS_car_length + FLAGS_rear_axle_to_center;
    }

public:
    double FLAGS_car_width = 0.8;
    double FLAGS_car_length = 1.0;
    double FLAGS_safety_margin = 0;
    double FLAGS_circle_radius = sqrt(pow(FLAGS_car_length / 8, 2) + pow(FLAGS_car_width / 2, 2)) + FLAGS_safety_margin;
    double FLAGS_wheel_base = 0.64;
    double FLAGS_rear_axle_to_center = 0.32; //"distance from rear axle to vehicle center");
    double FLAGS_d1 = -3.0 / 8.0 * FLAGS_car_length + FLAGS_rear_axle_to_center;// "distance from rear axle to circle 1");
    double FLAGS_d2 = -1.0 / 8.0 * FLAGS_car_length + FLAGS_rear_axle_to_center;//, "distance from rear axle to circle 2");
    double FLAGS_d3 = 1.0 / 8.0 * FLAGS_car_length + FLAGS_rear_axle_to_center;//, "distance from rear axle to circle 3");
    double FLAGS_d4 = 3.0 / 8.0 * FLAGS_car_length + FLAGS_rear_axle_to_center;//, "distance from rear axle to circle 4");
    double FLAGS_max_steering_angle = 20.0 * M_PI / 180.0;//, "");
    double FLAGS_mu = 0.4;//, "friction param");
    double FLAGS_max_curvature_rate = 0.1;//, "max derivative of curvature");

    /////
    ///// Smoothing related.
    /////
    std::string FLAGS_smoothing_method = "TENSION2";// "rReference smoothing method");
    // bool ValidateSmoothingnMethod(const char *flagname, const std::string &value)
    // {
    //     return value == "ANGLE_DIFF" || value == "TENSION" || value == "TENSION2";
    // }
    //bool isSmoothingMethodValid = google::RegisterFlagValidator(&FLAGS_smoothing_method, ValidateSmoothingnMethod);
    //"IPOPT" or "OSQP"
    std::string FLAGS_tension_solver = "OSQP";//, "solver used in tension smoothing method");

    bool FLAGS_enable_searching = true;//, "search before optimization");

    double FLAGS_search_lateral_range = 20.0;//, "max offset when searching");

    double FLAGS_search_longitudial_spacing = 0.5;//, "longitudinal spacing when searching");

    double FLAGS_search_lateral_spacing = 1.6;//, "lateral spacing when searching");

    // TODO: change names!
    double FLAGS_frenet_angle_diff_weight = 1500;//, "frenet smoothing angle difference weight");

    double FLAGS_frenet_angle_diff_diff_weight = 200;//, "frenet smoothing angle diff diff weight");

    double FLAGS_frenet_deviation_weight = 15;//, "frenet smoothing deviation from the orignal path");

    double FLAGS_cartesian_curvature_weight = 1;//, "");

    double FLAGS_cartesian_curvature_rate_weight = 50;//, "");

    double FLAGS_cartesian_deviation_weight = 0.0;//, "");

    double FLAGS_tension_2_deviation_weight = 0.005;//, "");

    double FLAGS_tension_2_curvature_weight = 1;//, "");

    double FLAGS_tension_2_curvature_rate_weight = 10;//, "");

    bool FLAGS_enable_simple_boundary_decision = true;//, "faster, but may go wrong sometimes");

    double FLAGS_search_obstacle_cost = 0.4;//, "searching cost");

    double FLAGS_search_deviation_cost = 0.4;//, "offset from the original ref cost");
    /////

    ///// Optimization related
    /////
    std::string FLAGS_optimization_method = "KP";//, "optimization method, named by input: " \
                                         "K uses curvature as input, KP uses curvature' as input, and" \
                                         "KPC uses curvarure' and apply some constraints on it");
    // bool ValidateOptimizationMethod(const char *flagname, const std::string &value)
    // {
    //     return value == "K" || value == "KP" || value == "KPC";
    // }
    // bool isOptimizationMethodValid = google::RegisterFlagValidator &FLAGS_optimization_method, ValidateOptimizationMethod);

    double FLAGS_K_curvature_weight = 50;//, "curvature weight of solver K");

    double FLAGS_K_curvature_rate_weight = 200;//, "curvature rate weight of solver K");

    double FLAGS_K_deviation_weight = 0;//, "deviation weight of solver K");

    double FLAGS_KP_curvature_weight = 10;//, "curvature weight of solver KP and KPC");

    double FLAGS_KP_curvature_rate_weight = 200;//, "curvature rate weight of solver KP and KPC");

    double FLAGS_KP_deviation_weight = 0;//, "deviation weight of solver KP and KPC");

    double FLAGS_KP_slack_weight = 3;//, "punish distance to obstacles");

    double FLAGS_expected_safety_margin = 1.3;//, "soft constraint on the distance to obstacles");

    // TODO: make this work.
    bool FLAGS_constraint_end_heading = true;//, "add constraints on end heading");

    // TODO: make this work.
    bool FLAGS_enable_exact_position = false;//, "force the path to reach the exact goal state");
    /////

    ///// Others.
    /////
    bool FLAGS_enable_raw_output = true;//, "slower, better");

    double FLAGS_output_spacing = 0.3;//, "output interval");

    bool FLAGS_enable_computation_time_output = false;//, "output details on screen");

    bool FLAGS_enable_collision_check = true;//, "perform collision check before output");

    bool FLAGS_enable_dynamic_segmentation = true;//, "dense segmentation when the curvature is large.");
};
}
}
