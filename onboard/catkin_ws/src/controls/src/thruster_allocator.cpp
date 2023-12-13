#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>

#include "controls_utils.h"
#include "thruster_allocator.h"

ThrusterAllocator::ThrusterAllocator() {}

ThrusterAllocator::ThrusterAllocator(std::string wrench_file_path, std::string wrench_pinv_file_path)
{
    ControlsUtils::read_matrix_from_csv(wrench_file_path, wrench);
    ControlsUtils::read_matrix_from_csv(wrench_pinv_file_path, wrench_pinv);

    ROS_ASSERT_MSG(wrench.rows() == wrench_pinv.cols() && wrench.cols() == wrench_pinv.rows(),
                   "If wrench is m x n, wrench_pinv must be n x m.");
}

void ThrusterAllocator::allocate_thrusters(const Eigen::VectorXd &set_power, double &power_scale_factor,
                                           Eigen::VectorXd &unconstrained_allocs, Eigen::VectorXd &constrained_allocs,
                                           Eigen::VectorXd &actual_power)
{
    ROS_ASSERT_MSG(wrench_pinv.cols() == set_power.rows(), "Set power must have the same number of rows as wrench_pinv.");

    unconstrained_allocs = wrench_pinv * set_power;

    unconstrained_allocs *= power_scale_factor;

    // If maximum absolute value allocation is greater than the maximum allowed, normalize all allocations
    // so that the maximum absolute value allocation is the maximum allowed and ratios between all allocations
    // remain the same
    double downscale = 1.0;
    double allocs_max = unconstrained_allocs.array().abs().maxCoeff();
    if (allocs_max > this->max_alloc)
        downscale = allocs_max;

    constrained_allocs = Eigen::VectorXd::Zero(unconstrained_allocs.rows());
    for (int i = 0; i < unconstrained_allocs.rows(); i++)
        constrained_allocs(i) = unconstrained_allocs(i) / downscale;

    // Compute the power actually being delivered along each axis
    actual_power = wrench * constrained_allocs;
}
