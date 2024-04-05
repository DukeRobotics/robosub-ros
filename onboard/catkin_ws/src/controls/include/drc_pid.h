// Adapted from Andrew Zelenak's ROS PID Package.
// http://wiki.ros.org/pid
// https://bitbucket.org/AndyZe/pid/src/master

// Named DRC_PID to not conflict with ROS pid package
#ifndef DRC_PID_H
#define DRC_PID_H

#include <array>
#include <memory>

#include "controls_types.h"
#include "controls_utils.h"

class PID {
   private:
    // *****************************************************************************************************************
    // Non-configurable properties

    /**
     * @brief The maximum absolute value of the integral term. This is used to prevent integral windup.
     */
    double integral_clamp = 1000.0;

    /**
     * @brief The cutoff frequency of the butterworth filter used to filter the derivative term. The default value is
     *  1.0, which means that the filter will have a cutoff frequency at 1/4 of the sampling rate.
     */
    double c = 1.0;

    // *****************************************************************************************************************
    // Configurable properties

    /**
     * @brief Control effort will not be greater than this value.
     */
    double control_effort_max;

    /**
     * @brief Control effort will not be less than this value.
     */
    double control_effort_min;

    /**
     * @brief The type of derivative used to compute the derivative term.
     */
    PIDDerivativeTypesEnum derivative_type;

    /**
     * @brief The maximum absolute value rate at which the error can change per second. Must be non-negative. This is
     *  used to prevent large control efforts from being generated when the error changes rapidly. The error used for
     *  PID control will be clipped to the previous error +/- (error_ramp_rate * delta_time (in seconds)).
     */
    double error_ramp_rate;

    /**
     * @brief The weights of the proportional, integral, derviative, and feedforward terms. Changes to this map will be
     *  immediately reflected in the next iteration of the PID controller.
     */
    PIDGainsMap pid_gains;

    // *****************************************************************************************************************
    // State variables

    /**
     * @brief The integral of the error function. Updated every iteration of the PID controller.
     */
    double integral = 0;

    /**
     * @brief The three most recent errors, in order from newest to oldest.
     */
    std::array<double, 3> errors;

    /**
     * @brief The three most recent filtered errors, in order from newest to oldest.
     */
    std::array<double, 3> filtered_errors;

    /**
     * @brief The three most recent unfiltered calculated derivatives, in order from newest to oldest.
     */
    std::array<double, 3> derivs;

    /**
     * @brief The three most recent filtered derivatives, in order from newest to oldest. The type of derivative
     *  filtered is determined by `derivative_type`.
     */
    std::array<double, 3> filtered_derivs;

   public:
    /**
     * @brief Default constructor. All configurable properties are uninitialized except for `pid_gains`, in which
     *  all gains are set to zero.
     */
    PID();

    /**
     * @brief Construct a new PID object with the given parameters.
     *
     * @param control_effort_min The minimum control effort. Must be less than or equal to `control_effort_max`.
     * @param control_effort_max The maximum control effort. Must be greater than or equal to `control_effort_min`.
     * @param derivative_type The type of derivative used to compute the derivative term.
     * @param error_ramp_rate The maximum absolute value rate at which the error can change per second. Must be
     *  non-negative.
     * @param pid_gains The gains used to weight the PID terms. All four keys in the map must be present.
     */
    PID(const double &control_effort_min, const double &control_effort_max,
        const PIDDerivativeTypesEnum &derivative_type, const double &error_ramp_rate, const PIDGainsMap &pid_gains);

    /**
     * @brief Set the pid gains object.
     *
     * @param pid_gain_type The type of pid gain to set.
     */
    void set_pid_gain(const PIDGainTypesEnum &pid_gain_type, const double &value);

    /**
     * @brief Get the pid gains object.
     *
     * @return Constant reference to the pid gains object.
     */
    const PIDGainsMap &get_pid_gains() const;

    /**
     * @brief Reset PID controller in preparation for change in setpoint.
     */
    void reset();

    /**
     * @brief Second order butterworth filter. Used to reduce noise and maintain stability.
     *
     * @param values The three most recent unfiltered values, in order from newest to oldest.
     * @param filtered_values The three most recent filtered values, in order from newest to oldest. Only the second and
     *  third values are used.
     * @return The filtered value.
     */
    double second_order_butterworth(const std::array<double, 3> &values, const std::array<double, 3> &filtered_values);

    /**
     * @brief Run one iteration of the controller. `pid_gains` and `pid_terms` must be set before calling this function.
     *
     * @param error The difference between the setpoint and the current value. If angular, this should be in radians in
     *  the range [-pi, pi].
     * @param delta_time The time elapsed since the last iteration of the PID controller. Must be positive.
     * @param[out] info The PIDInfo object containing values computed by the PID controller.
     * @param provided_derivative The derivative of the error, used only if `derivative_type` is
     *  `PIDDerivativeTypesEnum::provided`.
     * @return The control effort.
     */
    double run(double error, double delta_time, PIDInfo &info, double provided_derivative = 0);
};

#endif