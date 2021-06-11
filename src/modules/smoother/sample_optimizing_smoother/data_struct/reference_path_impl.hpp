//
// Created by ljn on 20-3-23.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_IMPL_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_IMPL_HPP_
#include <vector>
#include <tuple>
#include <memory>
namespace bz_robot
{
namespace PathOptimizationNS {
class Map;
//class Config;
class State;
class CoveringCircleBounds;
namespace tk {
class spline;
}
class Configs;
class ReferencePathImpl {
 public:
    ReferencePathImpl(std::shared_ptr<Configs> p_configs);
    ~ReferencePathImpl();
    ReferencePathImpl(const ReferencePathImpl &ref) = delete;
    ReferencePathImpl &operator=(const ReferencePathImpl &ref) = delete;

    const tk::spline &getXS() const;
    const tk::spline &getYS() const;
    // Set smoothed reference path.
    void setSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    // Set search result. It's used to calculate boundaries.
    void setOriginalSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    const tk::spline &getOriginalXS() const;
    const tk::spline &getOriginalYS() const;
    void clear();
    bool trimStates();
    std::size_t getSize() const;
    double getLength() const;
    void setLength(double s);
    const std::vector<State> &getReferenceStates() const;
    const std::vector<CoveringCircleBounds> &getBounds() const;
    const std::vector<double> &getMaxKList() const;
    const std::vector<double> &getMaxKpList() const;
    std::vector<std::tuple<State, double, double>> display_abnormal_bounds() const;
    // Set reference_states_ directly, only used in solveWithoutSmoothing.
    void setReference(const std::vector<State> &reference);
    void setReference(const std::vector<State> &&reference);
    // Calculate upper and lower bounds for each covering circle.
    void updateBounds(std::shared_ptr<PathOptimizationNS::Map> p_map);
    void updateBoundsImproved(std::shared_ptr<PathOptimizationNS::Map> p_map);
    // If the reference_states_ have speed and acceleration information, call this func to calculate
    // curvature and curvature rate bounds.
    void updateLimits();
    // Calculate reference_states_ from x_s_ and y_s_, given delta s.
    bool buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger);

 private:
    std::vector<double> getClearanceWithDirectionStrict(const PathOptimizationNS::State &state,
                                                        std::shared_ptr<Map> p_map);
    State getApproxState(const State &original_state, const State &actual_state, double len) const;
    bool use_spline_{true};
    // Reference path spline representation.
    tk::spline *x_s_;
    tk::spline *y_s_;
    double max_s_{};
    tk::spline *original_x_s_;
    tk::spline *original_y_s_;
    double original_max_s_{};
    bool is_original_spline_set{false};
    // Divided smoothed path info.
    std::vector<State> reference_states_;
    std::vector<CoveringCircleBounds> bounds_;
    std::vector<double> max_k_list_;
    std::vector<double> max_kp_list_;
    // To test updateBounds function;
    std::vector<std::tuple<State, double, double>> display_set_;
    std::shared_ptr<Configs> mp_configs;
};
}
}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_DATA_STRUCT_REFERENCE_PATH_IMPL_HPP_
