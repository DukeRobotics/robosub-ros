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

void ThrusterAllocator::allocate_thrusters(const Eigen::VectorXd &set_power, double &power_multiplier,
                                           Eigen::VectorXd &allocs, Eigen::VectorXd &actual_power)
{
    ROS_ASSERT_MSG(wrench_pinv.cols() == set_power.rows(), "Set power must have the same number of rows as wrench_pinv.");

    allocs = wrench_pinv * set_power;

    allocs *= power_multiplier;

    // If maximum absolute value allocation is greater than the maximum allowed, normalize all allocations
    // so that the maximum absolute value allocation is the maximum allowed and ratios between all allocations
    // remain the same
    double allocs_max = allocs.array().abs().maxCoeff();
    if (allocs_max > this->max_alloc)
        allocs /= max_alloc;

    // Compute the power actually being delivered along each axis
    actual_power = wrench * allocs;
}
