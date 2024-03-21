#include "thruster_allocator.h"

#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

#include "controls_types.h"
#include "controls_utils.h"

ThrusterAllocator::ThrusterAllocator() {}

ThrusterAllocator::ThrusterAllocator(std::string wrench_file_path, std::string wrench_pinv_file_path) {
    // Read wrench and wrench_pinv from CSV files
    ControlsUtils::read_matrix_from_csv(wrench_file_path, wrench);
    ControlsUtils::read_matrix_from_csv(wrench_pinv_file_path, wrench_pinv);

    // Check that wrench and wrench_pinv have compatible dimensions
    ROS_ASSERT_MSG(wrench.rows() == wrench_pinv.cols() && wrench.cols() == wrench_pinv.rows(),
                   "If wrench is m x n, wrench_pinv must be n x m.");

    int num_thrusters = wrench.cols();
    Eigen::SparseMatrix<double> hessian = (wrench.transpose() * wrench).sparseView();
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(num_thrusters);
    Eigen::SparseMatrix<double> linearMatrix = Eigen::MatrixXd::Identity(num_thrusters, num_thrusters).sparseView();
    Eigen::VectorXd lower_bound = -Eigen::VectorXd::Ones(num_thrusters);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Ones(num_thrusters);

    solver.data()->setNumberOfVariables(num_thrusters);
    solver.data()->setNumberOfConstraints(num_thrusters);
    ROS_ASSERT_MSG(solver.data()->setHessianMatrix(hessian), "Thruster Allocator: Failed to set hessian matrix.");
    ROS_ASSERT_MSG(solver.data()->setGradient(gradient), "Thruster Allocator: Failed to set gradient vector.");
    ROS_ASSERT_MSG(solver.data()->setLinearConstraintsMatrix(linearMatrix),
                   "Thruster Allocator: Failed to set linear matrix.");
    ROS_ASSERT_MSG(solver.data()->setLowerBound(lower_bound), "Thruster Allocator: Failed to set lower bound.");
    ROS_ASSERT_MSG(solver.data()->setUpperBound(upper_bound), "Thruster Allocator: Failed to set upper bound.");

    solver.settings()->setPolish(true);
    solver.settings()->setVerbosity(false);

    ROS_ASSERT_MSG(solver.initSolver(), "Thruster Allocator: Failed to initialize solver.");
}

void ThrusterAllocator::get_pseudoinverse_solution(const Eigen::VectorXd &set_power,
                                                   Eigen::VectorXd &unconstrained_allocs) {
    unconstrained_allocs = wrench_pinv * set_power;
}

void ThrusterAllocator::get_qp_solution(const Eigen::VectorXd &set_power, Eigen::VectorXd &constrained_allocs) {
    // Update the gradient of the quadratic cost function
    Eigen::VectorXd updated_gradient = -wrench.transpose() * set_power;
    ROS_ASSERT_MSG(solver.updateGradient(updated_gradient), "Thruster Allocator: Failed to update gradient.");

    // Solve the quadratic programming problem and get the solution
    ROS_ASSERT_MSG(solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError,
                   "Thruster Allocator: Failed to solve QP problem.");
    constrained_allocs = solver.getSolution();
}

void ThrusterAllocator::clip_allocs(Eigen::VectorXd &allocs) {
    // Clip all components of `allocs` to range [-max_alloc, max_alloc]
    for (int i = 0; i < allocs.size(); i++) allocs(i) = ControlsUtils::clip(allocs(i), -max_alloc, max_alloc);
}

void ThrusterAllocator::allocate_thrusters(const Eigen::VectorXd &set_power, Eigen::VectorXd &unconstrained_allocs,
                                           Eigen::VectorXd &constrained_allocs, Eigen::VectorXd &actual_power,
                                           Eigen::VectorXd &power_disparity) {
    // Check that set_power and wrench_pinv have compatible dimensions
    ROS_ASSERT_MSG(wrench_pinv.cols() == set_power.rows(),
                   "Set power must have the same number of rows as wrench_pinv.");

    // Compute unconstrained thruster allocations
    get_pseudoinverse_solution(set_power, unconstrained_allocs);

    // If maximum absolute value allocation is greater than the maximum allowed, compute constrained allocations
    // using quadratic programming to minimize the difference between the set power and the actual power.
    // Avoid recomputing the solution if the maximum absolute value allocation is already within the limit.
    double allocs_max = unconstrained_allocs.array().abs().maxCoeff();
    if (allocs_max > max_alloc)
        get_qp_solution(set_power, constrained_allocs);
    else
        constrained_allocs = unconstrained_allocs;

    // Clip constrained allocs to guarantee that they are within the limits
    clip_allocs(constrained_allocs);

    // Compute the power that will actually be delivered to the thrusters with constrained allocations
    actual_power = wrench * constrained_allocs;

    // Compute the difference between the set power and the actual power
    power_disparity = set_power - actual_power;
}
