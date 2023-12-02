#ifndef PID_H
#define PID_H

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

    std::array<double, 3> errors;
    std::array<double, 3> filtered_errors;
    std::array<double, 3> derivs;
    std::array<double, 3> filtered_derivs;

    PID();
    PID(std::shared_ptr<PIDGainsMap> pid_gains);

    void reset();
    double clip(const double value, const double min, const double max);
    double second_order_butterworth(const std::array<double, 3> &values, const std::array<double, 3> &filtered_values);
    double run_loop(double error, double delta_time);
};

#endif