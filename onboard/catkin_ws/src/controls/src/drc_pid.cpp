#include <memory>
#include <ros/ros.h>

#include "drc_pid.h"
#include "controls_utils.h"

PID::PID(){};

PID::PID(std::shared_ptr<PIDGainsMap> pid_gains, std::shared_ptr<PIDGainsMap> pid_terms)
{
    this->pid_gains = pid_gains;
    this->pid_terms = pid_terms;
}

void PID::reset()
{
    // Reset integral to zero
    integral = 0.0;
}

double PID::clip(const double value, const double min, const double max)
{
    return std::max(min, std::min(value, max));
}

double PID::second_order_butterworth(const std::array<double, 3> &values, const std::array<double, 3> &filtered_values)
{
    // Reference: Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
    // See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html for a description of
    // the math. Implementation adapted from https://bitbucket.org/AndyZe/pid/src/master/src/pid.cpp#lines-271.
    return (1 / (1 + (c * c) + (1.414 * c))) *
           (values.at(2) + (2 * values.at(1)) + values.at(0) -
            ((c * c) - (1.414 * c) + 1) * filtered_values.at(2) -
            ((-2 * c * c) + 2) * filtered_values.at(1));
}

double PID::run_loop(double error, double delta_time)
{
    // If pid_gains is not set, return 0 and print error
    if (pid_gains == nullptr || pid_gains->size() == 0 || !pid_gains->count(PIDGainTypesEnum::KP) ||
        !pid_gains->count(PIDGainTypesEnum::KI) || !pid_gains->count(PIDGainTypesEnum::KD) ||
        !pid_gains->count(PIDGainTypesEnum::FF))
    {
        ROS_ERROR("PID gains not set.");
        return 0;
    }

    // If pid_terms is not set, return 0 and print error
    if (pid_terms == nullptr || pid_terms->size() == 0 || !pid_terms->count(PIDGainTypesEnum::KP) ||
        !pid_terms->count(PIDGainTypesEnum::KI) || !pid_terms->count(PIDGainTypesEnum::KD) ||
        !pid_terms->count(PIDGainTypesEnum::FF))
    {
        ROS_ERROR("PID terms not set.");
        return 0;
    }

    // If delta time is nonpositive, return 0 and print error
    if (delta_time <= 0)
    {
        ROS_WARN("Delta time must be positive.");
        return 0;
    }

    // Update errors
    // Don't need to deal with angular discontinuities as angular errors will always be in range [-pi, pi]
    errors.at(2) = errors.at(1);
    errors.at(1) = errors.at(0);
    errors.at(0) = error;

    // Update filtered errors
    filtered_errors.at(2) = filtered_errors.at(1);
    filtered_errors.at(1) = filtered_errors.at(0);
    filtered_errors.at(0) = second_order_butterworth(errors, filtered_errors);

    // Update integral
    integral += error * delta_time;

    // Integral windup protection against sudden setpoint changes or unactuatable states
    integral = clip(integral, -integral_clamp, integral_clamp);

    // Update derivatives
    derivs.at(2) = derivs.at(1);
    derivs.at(1) = derivs.at(0);
    derivs.at(0) = (errors.at(0) - errors.at(1)) / delta_time;

    // Update filtered derivatives
    filtered_derivs.at(2) = filtered_derivs.at(1);
    filtered_derivs.at(1) = filtered_derivs.at(0);
    filtered_derivs.at(0) = second_order_butterworth(derivs, filtered_derivs);

    // Calculate terms, weighted by their respective gains
    double p = pid_gains->at(PIDGainTypesEnum::KP) * filtered_errors.at(0);
    double i = pid_gains->at(PIDGainTypesEnum::KI) * integral;
    double d = pid_gains->at(PIDGainTypesEnum::KD) * filtered_derivs.at(0);
    double f = pid_gains->at(PIDGainTypesEnum::FF);
    double control_effort = p + i + d + f;

    // Update term values
    pid_terms->at(PIDGainTypesEnum::KP) = p;
    pid_terms->at(PIDGainTypesEnum::KI) = i;
    pid_terms->at(PIDGainTypesEnum::KD) = d;
    pid_terms->at(PIDGainTypesEnum::FF) = f;

    // Clip control effort to be within limits
    control_effort = clip(control_effort, control_effort_min, control_effort_max);

    return control_effort;
}
