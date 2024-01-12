// Named DRC_PID to not conflict with ROS pid package
#ifndef DRC_PID_H
#define DRC_PID_H

#include <memory>
#include <array>
#include <Eigen/Dense>
#include "controls_utils.h"

class PID
{
public:
    std::shared_ptr<PIDGainsMap> pid_gains;
    double integral_clamp = 1000.0;
    double c = 1.0;
    double integral = 0;
    double control_effort_max = 1.0;
    double control_effort_min = -1.0;

    std::array<double, 3> errors;
    std::array<double, 3> filtered_errors;
    std::array<double, 3> derivs;
    std::array<double, 3> filtered_derivs;

    std::shared_ptr<PIDGainsMap> pid_terms;

    PID();
    PID(std::shared_ptr<PIDGainsMap> pid_gains, std::shared_ptr<PIDGainsMap> pid_terms);

    void reset();
    double clip(const double value, const double min, const double max);
    double second_order_butterworth(const std::array<double, 3> &values, const std::array<double, 3> &filtered_values);
    double run_loop(double error, double delta_time);
};

#endif