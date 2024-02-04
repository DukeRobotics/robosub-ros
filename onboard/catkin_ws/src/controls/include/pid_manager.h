#ifndef PID_MANAGER_H
#define PID_MANAGER_H

#include <memory>
#include <unordered_map>

#include "controls_utils.h"
#include "drc_pid.h"

class PIDManager
{
private:
    /**
     * @brief PID controllers for each axis
     */
    std::unordered_map<AxesEnum, PID> pid_controllers;

public:
    /**
     * @brief Construct PID controllers with uninitialized properties.
     */
    PIDManager();

    /**
     * @brief Construct PID controllers with specified properties.
     *
     * @param control_effort_limit Maximum absolute value control effort for each axis.
     * @param derivative_type Derivative type for each axis.
     * @param error_ramp_rate Maximum rate of change of error per second for each axis.
     * @param pid_gains PID gains for each axis.
     */
    PIDManager(const AxesMap<double> &control_effort_limit, const AxesMap<PIDDerivativeTypesEnum> &derivative_type,
               const AxesMap<double> &error_ramp_rate, const AxesMap<PIDGainsMap> &pid_gains);

    /**
     * @brief Set the pid gains for all axes.
     *
     * @param axis Axis to set PID gains for.
     * @param pid_gain_type PID gain type to set.
     * @param value Value to set the PID gain to.
     */
    void set_pid_gain(const AxesEnum &axis, const PIDGainTypesEnum &pid_gain_type, const double &value);

    /**
     * @brief Get the pid gains object
     *
     * @return Constant reference to the pid gains object.
     */
    AxesMap<PIDGainsMap> get_axes_pid_gains() const;

    /**
     * @brief Run PID loops
     *
     * @param errors Errors for each axis
     * @param deltaTimes Delta times for each axis
     * @param[out] outputs PID outputs for each axis
     */
    void run_loops(const AxesMap<double> &errors, const AxesMap<double> &deltaTimes, AxesMap<double> &outputs);

    /**
     * @brief Reset PID loop for a given axis
     *
     * @param axis_to_reset Axis to reset
     */
    void reset(AxesEnum axis_to_reset);
};

#endif