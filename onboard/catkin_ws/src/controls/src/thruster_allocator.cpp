#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>

#include "controls_utils.h"
#include "thruster_allocator.h"

ThrusterAllocator::ThrusterAllocator() {}

ThrusterAllocator::ThrusterAllocator(std::string wrench_file_path, std::string wrench_pinv_file_path)
{
    // Read wrench and wrench_pinv from CSV files
    ControlsUtils::read_matrix_from_csv(wrench_file_path, wrench);
    ControlsUtils::read_matrix_from_csv(wrench_pinv_file_path, wrench_pinv);

    // Check that wrench and wrench_pinv have compatible dimensions
    ROS_ASSERT_MSG(wrench.rows() == wrench_pinv.cols() && wrench.cols() == wrench_pinv.rows(),
                   "If wrench is m x n, wrench_pinv must be n x m.");
}

void ThrusterAllocator::allocate_thrusters(const Eigen::VectorXd &set_power, double &power_scale_factor,
                                           Eigen::VectorXd &set_power_scaled, Eigen::VectorXd &unconstrained_allocs,
                                           Eigen::VectorXd &constrained_allocs, Eigen::VectorXd &actual_power)
{
    // Check that set_power and wrench_pinv have compatible dimensions
    ROS_ASSERT_MSG(wrench_pinv.cols() == set_power.rows(), "Set power must have the same number of rows as wrench_pinv.");

    // Scale set power by power scale factor
    set_power_scaled = set_power * power_scale_factor;

    // Compute unconstrained thruster allocations
    unconstrained_allocs = wrench_pinv * set_power_scaled;

    // If maximum absolute value allocation is greater than the maximum allowed, normalize all allocations
    // so that the maximum absolute value allocation is the maximum allowed and ratios between all allocations
    // remain the same
    double downscale = 1.0;
    double allocs_max = unconstrained_allocs.array().abs().maxCoeff();
    if (allocs_max > this->max_alloc)
        downscale = allocs_max;

    // Compute constrained thruster allocations by scaling unconstrained allocations
    constrained_allocs = Eigen::VectorXd::Zero(unconstrained_allocs.rows());
    for (int i = 0; i < unconstrained_allocs.rows(); i++)
        constrained_allocs(i) = unconstrained_allocs(i) / downscale;

    // Compute the power that will actually be delivered to the thrusters with constrained allocations
    actual_power = wrench * constrained_allocs;
}
