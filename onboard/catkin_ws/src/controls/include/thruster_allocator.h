#ifndef THRUSTER_ALLOCATOR_H
#define THRUSTER_ALLOCATOR_H

#include <Eigen/Dense>
#include <string>
#include <tuple>

class ThrusterAllocator
{
    private:
        void read_matrix_from_csv(std::string file_path, Eigen::MatrixXd *matrix);

    public:
        Eigen::MatrixXd wrench;
        Eigen::MatrixXd wrench_pinv;
        ThrusterAllocator();
        ThrusterAllocator(std::string wrench_file_path, std::string wrench_pinv_file_path);
        void allocate_thrusters(Eigen::VectorXd set_power, Eigen::VectorXd *allocs, Eigen::VectorXd *actual_power);
};

#endif