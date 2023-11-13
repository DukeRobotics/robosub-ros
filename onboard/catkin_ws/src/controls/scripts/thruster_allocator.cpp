#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "thruster_allocator.h"

void ThrusterAllocator::read_matrix_from_csv(std::string file_path, Eigen::MatrixXd *matrix)
{
    // Open the CSV file
    std::ifstream file(file_path);

    if (!file.is_open())
    {
        std::cerr << "Error opening the file!" << std::endl;
    }

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

ThrusterAllocator::ThrusterAllocator(std::string wrench_file_path, std::string wrench_pinv_file_path)
{
    read_matrix_from_csv(wrench_file_path, &wrench);
    read_matrix_from_csv(wrench_pinv_file_path, &wrench_pinv);
}

void ThrusterAllocator::allocate_thrusters(Eigen::VectorXd set_power, Eigen::VectorXd *allocs, Eigen::VectorXd *actual_power)
{
    assert((set_power.rows() == 6 && set_power.cols() == 1, "Set power vector must be 6 x 1."));

    *allocs = wrench_pinv * set_power;

    // If maximum absolute value allocation is greater than 1, normalize all allocations
    // so that the maximum absolute value allocation is 1 and ratios between all allocations remain the same
    double max_alloc = allocs->array().abs().maxCoeff();
    if (max_alloc > 1)
    {
        *allocs /= max_alloc;
    }

    *actual_power = wrench * (*allocs);
}

int main(int argc, char **argv)
{
    ThrusterAllocator thruster_allocator = ThrusterAllocator("../config/oogway_wrench.csv", "../config/oogway_wrench_pinv.csv");
    std::cout << thruster_allocator.wrench << std::endl;
    std::cout << thruster_allocator.wrench_pinv << std::endl;
    return 0;
}
