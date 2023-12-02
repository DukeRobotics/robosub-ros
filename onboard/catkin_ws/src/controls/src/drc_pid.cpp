#include <memory>
#include <ros/ros.h>
#include "drc_pid.h"
#include "controls_utils.h"

PID::PID(){};

PID::PID(std::shared_ptr<PIDGainsMap> pid_gains)
{
    this->pid_gains = pid_gains;
}

void PID::reset()
{
    integral = 0.0;
}

double PID::clip(const double value, const double min, const double max)
{
    return std::max(min, std::min(value, max));
}

// Implementation of second order butterworth filter from Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
// See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html
double PID::second_order_butterworth(const std::array<double, 3> &values, const std::array<double, 3> &filtered_values)
{
    return (1 / (1 + (c * c) + (1.414 * c))) *
           (values.at(2) + (2 * values.at(1)) + values.at(0) -
            ((c * c) - (1.414 * c) + 1) * filtered_values.at(2) -
            ((-2 * c * c) + 2) * filtered_values.at(1));
}

double PID::run_loop(double error, double delta_time)
{
    // If pid_gains is not set, return 0 and print error
    if (pid_gains == nullptr || pid_gains->size() == 0 || !pid_gains->count(PIDGainTypesEnum::KP) ||
        !pid_gains->count(PIDGainTypesEnum::KI) || !pid_gains->count(PIDGainTypesEnum::KD))
    {
        ROS_ERROR("PID gains not set.");
        return 0;
    }

    // If delta time is nonpositive, return 0 and print error
    if (delta_time <= 0)
    {
        ROS_WARN("Delta time must be positive.");
        return 0;
    }

    errors.at(2) = errors.at(1);
    errors.at(1) = errors.at(0);
    errors.at(0) = error;

    // Angular errors are in radians
    // Don't need to deal with angular discontinuities as angles will always be in range [-pi, pi]

    // Update integral
    integral += error * delta_time;

    // Integral windup protection against sudden setpoint changes or unactuatable states
    integral = clip(integral, -integral_clamp, integral_clamp);

    // Update filtered error
    filtered_errors.at(2) = filtered_errors.at(1);
    filtered_errors.at(1) = filtered_errors.at(0);
    filtered_errors.at(0) = second_order_butterworth(errors, filtered_errors);

    // Update derivative
    derivs.at(2) = derivs.at(1);
    derivs.at(1) = derivs.at(0);
    derivs.at(0) = (errors.at(0) - errors.at(1)) / delta_time;

    // Update filtered derivative
    filtered_derivs.at(2) = filtered_derivs.at(1);
    filtered_derivs.at(1) = filtered_derivs.at(0);
    filtered_derivs.at(0) = second_order_butterworth(derivs, filtered_derivs);

    // Calculate PID output
    double p = pid_gains->at(PIDGainTypesEnum::KP) * filtered_errors.at(0);
    double i = pid_gains->at(PIDGainTypesEnum::KI) * integral;
    double d = pid_gains->at(PIDGainTypesEnum::KD) * filtered_derivs.at(0);
    double f = pid_gains->at(PIDGainTypesEnum::FF);
    double control_effort = p + i + d + f;

    // Clip control effort to be within limits
    control_effort = clip(control_effort, control_effort_min, control_effort_max);

    return control_effort;
}
