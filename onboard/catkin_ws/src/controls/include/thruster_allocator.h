#ifndef THRUSTER_ALLOCATOR_H
#define THRUSTER_ALLOCATOR_H

#include <Eigen/Dense>
#include <string>
#include <tuple>

class ThrusterAllocator
{
public:
    double max_alloc = 1.0;
    Eigen::MatrixXd wrench;
    Eigen::MatrixXd wrench_pinv;
    ThrusterAllocator();
    ThrusterAllocator(std::string wrench_file_path, std::string wrench_pinv_file_path);
    void allocate_thrusters(const Eigen::VectorXd &set_power, double &power_multiplier,
                            Eigen::VectorXd &unconstrained_allocs, Eigen::VectorXd &constrained_allocs,
                            Eigen::VectorXd &actual_power);
};

#endif