//
// Created by ljn on 20-3-23.
//
#include "sample_optimizing_smoother/data_struct/reference_path_impl.hpp"
#include "sample_optimizing_smoother/data_struct/reference_path.hpp"
#include "sample_optimizing_smoother/data_struct/data_struct.hpp"
#include "sample_optimizing_smoother/tools/spline.h"
#include "sample_optimizing_smoother/tools/tools.hpp"
#include "sample_optimizing_smoother/tools/Map.hpp"
namespace bz_robot
{
namespace PathOptimizationNS {

ReferencePath::ReferencePath(std::shared_ptr<Configs> p_configs) :
    mp_configs(p_configs),
    reference_path_impl_(std::make_shared<ReferencePathImpl>(p_configs))
{

}

const tk::spline &ReferencePath::getXS() const {
    return reference_path_impl_->getXS();
}

const tk::spline &ReferencePath::getYS() const {
    return reference_path_impl_->getYS();
}

double ReferencePath::getXS(double s) const {
    return reference_path_impl_->getXS()(s);
}

double ReferencePath::getYS(double s) const {
    return reference_path_impl_->getYS()(s);
}

void ReferencePath::clear() {
    reference_path_impl_->clear();
}

std::size_t ReferencePath::getSize() const {
    return reference_path_impl_->getSize();
}

double ReferencePath::getLength() const {
    return reference_path_impl_->getLength();
}

void ReferencePath::setLength(double s) {
    reference_path_impl_->setLength(s);
}

const std::vector<State> &ReferencePath::getReferenceStates() const {
    return reference_path_impl_->getReferenceStates();
}

const std::vector<CoveringCircleBounds> &ReferencePath::getBounds() const {
    return reference_path_impl_->getBounds();
}

const std::vector<double> &ReferencePath::getMaxKList() const {
    return reference_path_impl_->getMaxKList();
}

const std::vector<double> &ReferencePath::getMaxKpList() const {
    return reference_path_impl_->getMaxKpList();
}

std::vector<std::tuple<State, double, double>> ReferencePath::display_abnormal_bounds() const {
    return reference_path_impl_->display_abnormal_bounds();
}

void ReferencePath::setReference(const std::vector<PathOptimizationNS::State> &reference) {
    reference_path_impl_->setReference(reference);
}

void ReferencePath::setReference(const std::vector<PathOptimizationNS::State> &&reference) {
    reference_path_impl_->setReference(reference);
}

void ReferencePath::updateBounds(std::shared_ptr<Map> p_map) {
    reference_path_impl_->updateBoundsImproved(p_map);
}

void ReferencePath::updateLimits() {
    reference_path_impl_->updateLimits();
}

bool ReferencePath::buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger) {
    reference_path_impl_->buildReferenceFromSpline(delta_s_smaller, delta_s_larger);
}

void ReferencePath::setSpline(const PathOptimizationNS::tk::spline &x_s,
                              const PathOptimizationNS::tk::spline &y_s,
                              double max_s) {
    reference_path_impl_->setSpline(x_s, y_s, max_s);
}

void ReferencePath::setOriginalSpline(const PathOptimizationNS::tk::spline &x_s,
                                      const PathOptimizationNS::tk::spline &y_s,
                                      double max_s) {
    reference_path_impl_->setOriginalSpline(x_s, y_s, max_s);
}

}
}
