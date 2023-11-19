#include <memory>
#include "pid.h"

PID::PID() {};

PID::PID(std::shared_ptr<PIDGainsMap> pid_gains,
         double integral_clamp,
         double cutoff_freq,
         bool angle_correction)
{
    this->pid_gains = pid_gains;
    this->integral_clamp = integral_clamp;
    this->cutoff_freq = cutoff_freq;
    this->angle_correction = angle_correction;
}

void PID::reset()
{
    integral = 0.0;
    prev_values = Eigen::Matrix<double, 3, 3>::Zero();
}

double PID::run_loop(double error, double deltaTime)
{
    // TODO: Implement PID controller
    return 0;
}

