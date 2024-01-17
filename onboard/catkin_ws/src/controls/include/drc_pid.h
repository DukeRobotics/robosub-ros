// Adapted from Andrew Zelenak's ROS PID Package.
// http://wiki.ros.org/pid
// https://bitbucket.org/AndyZe/pid/src/master

// Named DRC_PID to not conflict with ROS pid package
#ifndef DRC_PID_H
#define DRC_PID_H

#include <array>
#include <memory>

#include "controls_utils.h"

class PID
{
public:

    /**
     * @brief The maximum absolute value of the integral term. This is used to prevent integral windup.
     */
    double integral_clamp = 1000.0;

    /**
     * @brief The cutoff frequency of the butterworth filter used to filter the derivative term. The default value is
     *  1.0, which means that the filter will have a cutoff frequency at 1/4 of the sampling rate.
     */
    double c = 1.0;

    /**
     * @brief Control effort will not be greater than this value.
     */
    double control_effort_max = 1.0;

    /**
     * @brief Control effort will not be less than this value.
     */
    double control_effort_min = -1.0;

    /**
     * @brief The integral of the error function. Updated every iteration of the PID loop.
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
     * @brief The three most recent unfiltered derivatives, in order from newest to oldest.
     */
    std::array<double, 3> derivs;

    /**
     * @brief The three most recent filtered derivatives, in order from newest to oldest.
     */
    std::array<double, 3> filtered_derivs;

    /**
     * @brief The weights of the proportional, integral, derviative, and feedforward terms. Updated _only_ by
     *  instantiators of the PID class. Changes to this map will be immediately reflected in the next iteration of the
     *  PID loop.
     */
    std::shared_ptr<PIDGainsMap> pid_gains;

    /**
     * @brief The proportional, integral, derviative, and feedforward terms that are summed to get the control effort.
     *  Updated every iteration of the PID loop.
     */
    std::shared_ptr<PIDGainsMap> pid_terms;

    /**
     * @brief Construct a new PID object with `pid_gains` and `pid_terms` set to `nullptr`.
     */
    PID();

    /**
     * @brief Construct a new PID object with `pid_gains` and `pid_terms` set to the given pointers.
     *
     * @param pid_gains The gains that will be updated by instantiators of this class and used to weight the PID terms.
     * @param pid_terms The terms that will be updated by this class and summed to get the control effort.
     */
    PID(std::shared_ptr<PIDGainsMap> pid_gains, std::shared_ptr<PIDGainsMap> pid_terms);

    /**
     * @brief Reset PID loop in preparation for change in setpoint.
    */
    void reset();

    /**
     * @brief Clip value between `min` and `max`. If `value` is less than `min`, return `min`. If `value` is greater
     *  than `max`, return `max`. Otherwise, return `value`.
     *
     * @param value The value to clip.
     * @param min The minimum value.
     * @param max The maximum value.
     * @return The clipped value, guaranteed to be between `min` and `max` (inclusive).
     */
    double clip(const double value, const double min, const double max);

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
     * @brief Run one iteration of the PID loop. `pid_gains` and `pid_terms` must be set before calling this function.
     *
     * @param error The difference between the setpoint and the current value. If angular, this should be in radians in
     *  the range [-pi, pi].
     * @param delta_time The time elapsed since the last iteration of the PID loop. Must be positive.
     * @return The control effort.
     */
    double run_loop(double error, double delta_time);
};

#endif