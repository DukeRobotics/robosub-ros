#include "drc_pid.h"

#include <ros/ros.h>

#include <memory>

#include "controls_types.h"
#include "controls_utils.h"

PID::PID() {
    // Set all pid gains to zero
    this->pid_gains = {{PIDGainTypesEnum::KP, 0.0},
                       {PIDGainTypesEnum::KI, 0.0},
                       {PIDGainTypesEnum::KD, 0.0},
                       {PIDGainTypesEnum::FF, 0.0}};
};

PID::PID(const double &control_effort_min, const double &control_effort_max,
         const PIDDerivativeTypesEnum &derivative_type, const double &error_ramp_rate, const PIDGainsMap &pid_gains) {
    // Validate inputs
    ROS_ASSERT_MSG(control_effort_min <= control_effort_max,
                   "PID initialization error: Control effort min must be less than or equal to control effort max.");
    ROS_ASSERT_MSG(ControlsUtils::pid_gains_map_valid(pid_gains),
                   "PID initialization error: PID gains map is invalid.");
    ROS_ASSERT_MSG(error_ramp_rate >= 0, "PID initialization error: Error ramp rate must be non-negative.");
    ROS_ASSERT_MSG(ControlsUtils::value_in_pid_derivative_types_enum(derivative_type),
                   "PID initialization error: Derivative type is invalid.");

    // Set parameters
    this->control_effort_min = control_effort_min;
    this->control_effort_max = control_effort_max;
    this->derivative_type = derivative_type;
    this->error_ramp_rate = error_ramp_rate;
    this->pid_gains = pid_gains;
}

void PID::set_pid_gain(const PIDGainTypesEnum &pid_gain_type, const double &value) {
    // Set gains
    this->pid_gains[pid_gain_type] = value;
}

const PIDGainsMap &PID::get_pid_gains() const { return pid_gains; }

void PID::reset() {
    // Reset integral to zero
    integral = 0.0;
}

double PID::second_order_butterworth(const std::array<double, 3> &values,
                                     const std::array<double, 3> &filtered_values) {
    // Reference: Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
    // See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html for a description of
    // the math. Implementation adapted from https://bitbucket.org/AndyZe/pid/src/master/src/pid.cpp#lines-271.
    return (1 / (1 + (c * c) + (1.414 * c))) *
           (values.at(2) + (2 * values.at(1)) + values.at(0) - ((c * c) - (1.414 * c) + 1) * filtered_values.at(2) -
            ((-2 * c * c) + 2) * filtered_values.at(1));
}

double PID::run(double error, double delta_time, PIDInfo &info, double provided_derivative) {
    // If there are validation errors, an exception is not thrown to maintain continuous operation of controls in the
    // event of a temporary error. Instead, the function returns 0 and prints an error message.

    // If delta time is nonpositive, return 0 and print error
    if (delta_time <= 0) {
        ROS_WARN("Delta time must be positive.");
        return 0;
    }

    // Update errors
    // Don't need to deal with angular discontinuities as angular errors will always be in range [-pi, pi]
    errors.at(2) = errors.at(1);
    errors.at(1) = errors.at(0);

    // Clip error to be within error ramp rate
    double max_error_change = error_ramp_rate * delta_time;
    errors.at(0) = ControlsUtils::clip(error, errors.at(1) - max_error_change, errors.at(1) + max_error_change);

    // Update filtered errors
    filtered_errors.at(2) = filtered_errors.at(1);
    filtered_errors.at(1) = filtered_errors.at(0);
    filtered_errors.at(0) = second_order_butterworth(errors, filtered_errors);

    // Update integral
    integral += error * delta_time;

    // Integral windup protection against sudden setpoint changes or unactuatable states
    integral = ControlsUtils::clip(integral, -integral_clamp, integral_clamp);

    // Update derivatives
    derivs.at(2) = derivs.at(1);
    derivs.at(1) = derivs.at(0);

    // Calculate derivative
    double calculated_derivative = (errors.at(0) - errors.at(1)) / delta_time;

    // Update derivative based on derivative type
    if (derivative_type == PIDDerivativeTypesEnum::CALCULATED)
        derivs.at(0) = calculated_derivative;

    else if (derivative_type == PIDDerivativeTypesEnum::PROVIDED)
        derivs.at(0) = provided_derivative;

    else {
        // Print error and return 0 if derivative type is invalid
        ROS_ERROR("PID run loop error: Derivative type is invalid.");
        return 0;
    }

    // Update filtered derivatives
    filtered_derivs.at(2) = filtered_derivs.at(1);
    filtered_derivs.at(1) = filtered_derivs.at(0);
    filtered_derivs.at(0) = second_order_butterworth(derivs, filtered_derivs);

    // Calculate terms, weighted by their respective gains
    double p = pid_gains.at(PIDGainTypesEnum::KP) * filtered_errors.at(0);
    double i = pid_gains.at(PIDGainTypesEnum::KI) * integral;
    double d = pid_gains.at(PIDGainTypesEnum::KD) * filtered_derivs.at(0);
    double f = pid_gains.at(PIDGainTypesEnum::FF);
    double control_effort = p + i + d + f;

    // Update PIDInfo
    info.terms.proportional = p;
    info.terms.integral = i;
    info.terms.derivative = d;
    info.terms.feedforward = f;

    info.filtered_error = filtered_errors.at(0);
    info.integral = integral;
    info.filtered_derivative = filtered_derivs.at(0);

    info.calculated_derivative = calculated_derivative;
    info.provided_derivative = provided_derivative;
    info.derivative_type = derivative_type;

    // Clip control effort to be within limits
    control_effort = ControlsUtils::clip(control_effort, control_effort_min, control_effort_max);

    return control_effort;
}