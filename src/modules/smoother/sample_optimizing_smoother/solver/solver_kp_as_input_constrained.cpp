//
// Created by ljn on 20-3-10.
//

#include "sample_optimizing_smoother/solver/solver_kp_as_input_constrained.hpp"
#include "sample_optimizing_smoother/data_struct/data_struct.hpp"
#include "sample_optimizing_smoother/data_struct/reference_path.hpp"
#include "sample_optimizing_smoother/data_struct/vehicle_state_frenet.hpp"
#include "sample_optimizing_smoother/tools/tools.hpp"
#include "sample_optimizing_smoother/config/planning_flags.hpp"

namespace bz_robot
{
namespace PathOptimizationNS {
SolverKpAsInputConstrained::SolverKpAsInputConstrained(const ReferencePath &reference_path,
                                                       const VehicleState &vehicle_state,
                                                       const size_t &horizon,
                                                       std::shared_ptr<Configs> p_configs) :
    OsqpSolver(reference_path, vehicle_state, horizon, p_configs),
    keep_control_steps_(4), // TODO: adjust this.
    control_horizon_((horizon_ + keep_control_steps_ - 2) / keep_control_steps_),
    state_size_(3 * horizon_),
    control_size_(control_horizon_),
    slack_size_(3 * horizon_) {
    num_of_variables_ = state_size_ + control_size_ + slack_size_;
    num_of_constraints_ = 12 * horizon_ + 3 * control_horizon_ + 2;
}

void SolverKpAsInputConstrained::getOptimizedPath(const Eigen::VectorXd &optimization_result,
                                                  std::vector<PathOptimizationNS::State> *optimized_path) const {
    //CHECK_EQ(optimization_result.size(), num_of_variables_);
    const auto &ref_states = reference_path_.getReferenceStates();
    optimized_path->clear();
    double tmp_s = 0;
    for (size_t i = 0; i != horizon_; ++i) {
        double angle = ref_states[i].z;
        double new_angle = constraintAngle(angle + M_PI_2);
        double tmp_x = ref_states[i].x + optimization_result(3 * i) * cos(new_angle);
        double tmp_y = ref_states[i].y + optimization_result(3 * i) * sin(new_angle);
        double k = optimization_result(3 * i + 2);
        if (i != 0) {
            tmp_s += sqrt(pow(tmp_x - optimized_path->back().x, 2) + pow(tmp_y - optimized_path->back().y, 2));
        }
        optimized_path->emplace_back(tmp_x, tmp_y, angle + optimization_result(3 * i + 1), k, tmp_s);
    }
}

void SolverKpAsInputConstrained::setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const {
    const size_t matrix_size = state_size_ + control_size_ + slack_size_;
    Eigen::MatrixXd hessian{Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0)};
    double w_c = mp_configs->FLAGS_KP_curvature_weight;
    double w_cr = mp_configs->FLAGS_KP_curvature_rate_weight;
    double w_pq = mp_configs->FLAGS_KP_deviation_weight;
    double w_collision_slack = mp_configs->FLAGS_KP_slack_weight;
    double w_k_slack = 500;
    double w_kp_slack = 25000;
    for (size_t i = 0; i != horizon_; ++i) {
        hessian(3 * i, 3 * i) += w_pq;
        hessian(3 * i + 2, 3 * i + 2) += w_c;
        hessian(state_size_ + control_size_ + i, state_size_ + control_size_ + i) += w_collision_slack;
        hessian(state_size_ + control_size_ + horizon_ + i, state_size_ + control_size_ + horizon_ + i) += w_k_slack;
    }
    for (size_t i = 0; i != control_horizon_; ++i) {
        hessian(state_size_ + i, state_size_ + i) += keep_control_steps_ * w_cr;
        hessian(state_size_ + control_size_ + 2 * horizon_ + i, state_size_ + control_size_ + 2 * horizon_ + i) +=
            w_kp_slack * keep_control_steps_;
    }
    *matrix_h = hessian.sparseView();
}

void SolverKpAsInputConstrained::setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                                                     Eigen::VectorXd *lower_bound,
                                                     Eigen::VectorXd *upper_bound) const {
    const auto &ref_states = reference_path_.getReferenceStates();
    const size_t trans_range_begin{0};
    const size_t kl_range_begin{trans_range_begin + 3 * horizon_}; // k lower
    const size_t ku_range_begin{kl_range_begin + horizon_}; // k upper
    const size_t kpl_range_begin{ku_range_begin + horizon_}; // kp lower
    const size_t kpu_range_begin{kpl_range_begin + control_horizon_}; // kp upper
    const size_t slack_range_begin{kpu_range_begin + control_horizon_};
    const size_t collision_range_begin{slack_range_begin + 2 * horizon_ + control_horizon_};
    const size_t end_state_range_begin{collision_range_begin + 5 * horizon_};
    Eigen::MatrixXd cons =
        Eigen::MatrixXd::Zero(12 * horizon_ + 3 * control_horizon_ + 2, state_size_ + control_size_ + slack_size_);
    // Set transition part.
    for (size_t i = 0; i != state_size_; ++i) {
        cons(i, i) = -1;
    }
    Eigen::Matrix3d a(Eigen::Matrix3d::Zero());
    a(0, 1) = 1;
    a(1, 2) = 1;
    Eigen::Matrix<double, 3, 1> b(Eigen::MatrixXd::Constant(3, 1, 0));
    b(2, 0) = 1;
    std::vector<Eigen::MatrixXd> c_list;
    for (size_t i = 0; i != horizon_ - 1; ++i) {
        const auto ref_k{ref_states[i].k};
        const auto ds{ref_states[i + 1].s - ref_states[i].s};
        const auto ref_kp{(ref_states[i + 1].k - ref_k) / ds};
        a(1, 0) = -pow(ref_k, 2);
        auto A{a * ds + Eigen::Matrix3d::Identity()};
        auto B{b * ds};
        cons.block(3 * (i + 1), 3 * i, 3, 3) = A;
        size_t control_index{i / keep_control_steps_};
        cons.block(3 * (i + 1), state_size_ + control_index, 3, 1) = B;
        Eigen::Matrix<double, 3, 1> c, ref_state;
        c << 0, 0, ref_kp;
        ref_state << 0, 0, ref_k;
        c_list.emplace_back(ds * (c - a * ref_state - b * ref_kp));
    }

    // Set vars part.
    // kl and ku:
    for (size_t i = 0; i != horizon_; ++i) {
        cons(kl_range_begin + i, 3 * i + 2) = 1;
        cons(kl_range_begin + i, state_size_ + control_size_ + horizon_ + i) = 1;
        cons(ku_range_begin + i, 3 * i + 2) = 1;
        cons(ku_range_begin + i, state_size_ + control_size_ + horizon_ + i) = -1;
        cons(slack_range_begin + i, state_size_ + control_size_ + i) = 1;
        cons(slack_range_begin + horizon_ + i, state_size_ + control_size_ + horizon_ + i) = 1;
    }
    // kp:
    for (size_t i = 0; i != control_size_; ++i) {
        cons(kpl_range_begin + i, state_size_ + i) = 1;
        cons(kpl_range_begin + i, state_size_ + control_size_ + 2 * horizon_ + i) = 1;
        cons(kpu_range_begin + i, state_size_ + i) = 1;
        cons(kpu_range_begin + i, state_size_ + control_size_ + 2 * horizon_ + i) = -1;
        cons(slack_range_begin + 2 * horizon_ + i, state_size_ + control_size_ + 2 * horizon_ + i) = 1;
    }

    // Set collision part.
    Eigen::Matrix<double, 3, 2> collision;
    collision << 1, mp_configs->FLAGS_d1,
        1, mp_configs->FLAGS_d2,
//        1, mp_configs->FLAGS_d3,
        1, mp_configs->FLAGS_d4;
    for (size_t i = 0; i != horizon_; ++i) {
        cons.block(collision_range_begin + 3 * i, 3 * i, 3, 2) = collision;
    }
    Eigen::Matrix<double, 1, 2> collision1;
    collision1 << 1, mp_configs->FLAGS_d3;
    for (size_t i = 0; i != horizon_; ++i) {
        cons.block(collision_range_begin + 3 * horizon_ + i, 3 * i, 1, 2) = collision1;
        cons(collision_range_begin + 3 * horizon_ + i, state_size_ + control_size_ + i) = -1;
        cons.block(collision_range_begin + 4 * horizon_ + i, 3 * i, 1, 2) = collision1;
        cons(collision_range_begin + 4 * horizon_ + i, state_size_ + control_size_ + i) = 1;
    }

    // End state.
    cons(end_state_range_begin, state_size_ - 3) = 1; // end ey
    cons(end_state_range_begin + 1, state_size_ - 2) = 1; // end ephi
    *matrix_constraints = cons.sparseView();

    // Set bounds.
    *lower_bound = Eigen::MatrixXd::Zero(12 * horizon_ + 3 * control_horizon_ + 2, 1);
    *upper_bound = Eigen::MatrixXd::Zero(12 * horizon_ + 3 * control_horizon_ + 2, 1);
    Eigen::Matrix<double, 3, 1> x0;
    const auto init_error{vehicle_state_.getInitError()};
    x0 << init_error[0], init_error[1], vehicle_state_.getStartState().k;
    lower_bound->block(0, 0, 3, 1) = -x0;
    upper_bound->block(0, 0, 3, 1) = -x0;
    for (size_t i = 0; i != horizon_ - 1; ++i) {
        lower_bound->block(3 * (i + 1), 0, 3, 1) = -c_list[i];
        upper_bound->block(3 * (i + 1), 0, 3, 1) = -c_list[i];
    }

    // Vars bound.
    // kl and ku:
    for (size_t i = 0; i != horizon_; ++i) {
        const auto &max_k_list{reference_path_.getMaxKList()};
        (*lower_bound)(kl_range_begin + i) = -max_k_list[i];
        (*upper_bound)(kl_range_begin + i) = OsqpEigen::INFTY;
        (*lower_bound)(ku_range_begin + i) = -OsqpEigen::INFTY;
        (*upper_bound)(ku_range_begin + i) = max_k_list[i];

        (*lower_bound)(slack_range_begin + i) = 0;
        (*upper_bound)(slack_range_begin + i) = mp_configs->FLAGS_expected_safety_margin;
        (*lower_bound)(slack_range_begin + horizon_ + i) = 0;
        (*upper_bound)(slack_range_begin + horizon_ + i) = //OsqpEigen::INFTY;
            std::max(tan(mp_configs->FLAGS_max_steering_angle) / mp_configs->FLAGS_wheel_base - max_k_list[i], 0.0);
    }
    for (size_t i = 0; i != control_horizon_; ++i) {
        const auto &max_kp_list{reference_path_.getMaxKpList()};
        (*lower_bound)(kpl_range_begin + i) = -max_kp_list[i];
        (*upper_bound)(kpl_range_begin + i) = OsqpEigen::INFTY;
        (*lower_bound)(kpu_range_begin + i) = -OsqpEigen::INFTY;
        (*upper_bound)(kpu_range_begin + i) = max_kp_list[i];

        (*lower_bound)(slack_range_begin + 2 * horizon_ + i) = 0;
        (*upper_bound)(slack_range_begin + 2 * horizon_ + i) = OsqpEigen::INFTY;
    }

    // Collision bound.
    const auto &bounds = reference_path_.getBounds();
    for (size_t i = 0; i != horizon_; ++i) {
        Eigen::Vector3d ld, ud;
        ud
            << bounds[i].c0.ub, bounds[i].c1.ub, bounds[i].c3.ub;
        ld
            << bounds[i].c0.lb, bounds[i].c1.lb, bounds[i].c3.lb;
        lower_bound->block(collision_range_begin + 3 * i, 0, 3, 1) = ld;
        upper_bound->block(collision_range_begin + 3 * i, 0, 3, 1) = ud;
        double uds = bounds[i].c2.ub - mp_configs->FLAGS_expected_safety_margin;
        double lds = bounds[i].c2.lb + mp_configs->FLAGS_expected_safety_margin;
        (*upper_bound)(collision_range_begin + 3 * horizon_ + i, 0) = uds;
        (*lower_bound)(collision_range_begin + 3 * horizon_ + i, 0) = -OsqpEigen::INFTY;
        (*lower_bound)(collision_range_begin + 4 * horizon_ + i, 0) = lds;
        (*upper_bound)(collision_range_begin + 4 * horizon_ + i, 0) = OsqpEigen::INFTY;
    }

    // End state.
    // End ey is not constrained.
    (*lower_bound)(end_state_range_begin) = -OsqpEigen::INFTY;
    (*upper_bound)(end_state_range_begin) = OsqpEigen::INFTY;
    (*lower_bound)(end_state_range_begin + 1) = -OsqpEigen::INFTY;
    (*upper_bound)(end_state_range_begin + 1) = OsqpEigen::INFTY;
    if (mp_configs->FLAGS_constraint_end_heading) {
        double end_psi = constraintAngle(vehicle_state_.getEndState().z - ref_states.back().z);
        if (end_psi < 70 * M_PI / 180) {
            (*lower_bound)(end_state_range_begin + 1) = end_psi - 5 * M_PI / 180;
            (*upper_bound)(end_state_range_begin + 1) = end_psi + 5 * M_PI / 180;
        }
    }

}

}
}
