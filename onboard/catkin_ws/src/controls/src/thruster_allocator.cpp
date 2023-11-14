#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>

#include "thruster_allocator.h"

void ThrusterAllocator::read_matrix_from_csv(std::string file_path, Eigen::MatrixXd *matrix)
{
    // Open the CSV file
    std::ifstream file(file_path);

    if (!file.is_open())
        ROS_ASSERT_MSG(false, "Could not open file %s", file_path.c_str());

    // Read data from the CSV file and initialize the matrix
    std::vector<std::vector<double>> data;
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::vector<double> row;

        double value;
        char comma;

        while (iss >> value)
        {
            row.push_back(value);

            if (iss >> comma && comma != ',')
            {
                std::cerr << "Error: CSV format violation!" << std::endl;
            }
        }

        data.push_back(row);
    }

    // Close the file
    file.close();

    // Determine the matrix size
    int rows = data.size();
    int cols = (rows > 0) ? data[0].size() : 0;

    // Initialize the Eigen matrix
    matrix->resize(rows, cols);

    // Copy the data from the vector of vectors to the Eigen matrix
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            (*matrix)(i, j) = data[i][j];
        }
    }
}

ThrusterAllocator::ThrusterAllocator() {}

ThrusterAllocator::ThrusterAllocator(std::string wrench_file_path, std::string wrench_pinv_file_path)
{
    read_matrix_from_csv(wrench_file_path, &wrench);
    read_matrix_from_csv(wrench_pinv_file_path, &wrench_pinv);

    ROS_ASSERT_MSG(wrench.rows() == wrench_pinv.cols() && wrench.cols() == wrench_pinv.rows(),
                   "If wrench is m x n, wrench_pinv must be n x m.");
}

void ThrusterAllocator::allocate_thrusters(Eigen::VectorXd set_power, Eigen::VectorXd *allocs, Eigen::VectorXd *actual_power)
{
    ROS_ASSERT_MSG(wrench_pinv.rows() == set_power.rows(), "Set power must have the same number of rows as wrench_pinv.");

    *allocs = wrench_pinv * set_power;

    // If maximum absolute value allocation is greater than 1, normalize all allocations
    // so that the maximum absolute value allocation is 1 and ratios between all allocations remain the same
    double max_alloc = allocs->array().abs().maxCoeff();
    if (max_alloc > 1)
        *allocs /= max_alloc;

    // Compute the power actually being delivered along each axis
    *actual_power = wrench * (*allocs);
}
