#ifndef PID_MANAGER_H
#define PID_MANAGER_H

#include <memory>
#include <unordered_map>

#include "controls_types.h"
#include "controls_utils.h"
#include "drc_pid.h"

class PIDManager {
   private:
    /**
     * @brief PID controllers for each axis.
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
     * @param control_effort_mins Minimum control effort for each axis.
     * @param control_effort_maxes Maximum control effort for each axis.
     * @param derivative_types Derivative types for each axis.
     * @param error_ramp_rates Maximum rate of change of error per second for each axis.
     * @param pid_gains PID gains for each axis.
     */
    PIDManager(const AxesMap<double> &control_effort_mins, const AxesMap<double> &control_effort_maxes,
               const AxesMap<PIDDerivativeTypesEnum> &derivative_types, const AxesMap<double> &error_ramp_rates,
               const AxesMap<PIDGainsMap> &pid_gains);

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
     * @brief Run PID loop.
     *
     * @param errors Errors for each axis.
     * @param deltaTimes Delta times for each axis.
     * @param[out] outputs PID outputs for each axis.
     * @param[out] infos PID info for each axis.
     * @param derivatives Provided derivatives for each axis.
     */
    void run_loop(const AxesMap<double> &errors, const AxesMap<double> &deltaTimes, AxesMap<double> &outputs,
                  AxesMap<PIDInfo> &infos, const AxesMap<double> &derivatives = AxesMap<double>());

    /**
     * @brief Reset all PID controllers.
     */
    void reset();
};

#endif