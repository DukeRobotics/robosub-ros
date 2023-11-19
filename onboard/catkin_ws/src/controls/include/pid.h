#ifndef PID_H
#define PID_H

#include <memory>
#include <Eigen/Dense>
#include "controls_utils.h"

class PID
{
public:
    std::shared_ptr<PIDGainsMap> pid_gains;
    double integral_clamp;
    double cutoff_freq;
    bool angle_correction;
    Eigen::Matrix<double, 3, 3> prev_values;
    double integral;

    PID();
    PID(std::shared_ptr<PIDGainsMap> pid_gains,
        double integral_clamp = 1000.0,
        double cutoff_freq = 5.0,
        bool angle_correction = true);

    void reset();
    double run_loop(double error, double deltaTime);
};

#endif