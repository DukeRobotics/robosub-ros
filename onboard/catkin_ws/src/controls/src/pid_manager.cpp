#include <memory>
#include <unordered_map>

#include "controls_types.h"
#include "controls_utils.h"
#include "drc_pid.h"
#include "pid_manager.h"


PIDManager::PIDManager()
{
    // Initialize PID controllers with default values for parameters
    for (const AxesEnum &axis : AXES)
        pid_controllers[axis] = PID();
}

PIDManager::PIDManager(const AxesMap<double> &control_effort_limit,
                       const AxesMap<PIDDerivativeTypesEnum> &derivative_type, const AxesMap<double> &error_ramp_rate,
                       const AxesMap<PIDGainsMap> &pid_gains)
{
    // Initialize PID controllers with given parameters
    for (const AxesEnum &axis : AXES)
        pid_controllers[axis] = PID(control_effort_limit.at(axis), derivative_type.at(axis), error_ramp_rate.at(axis),
                                    pid_gains.at(axis));
}

void PIDManager::set_pid_gain(const AxesEnum &axis, const PIDGainTypesEnum &pid_gain_type, const double &value)
{
    // Set PID gains for given axis
    pid_controllers.at(axis).set_pid_gain(pid_gain_type, value);
}

AxesMap<PIDGainsMap> PIDManager::get_axes_pid_gains() const
{
    // Return PID gains for each axis
    AxesMap<PIDGainsMap> axes_pid_gains;
    for (const AxesEnum &axis : AXES)
        axes_pid_gains[axis] = pid_controllers.at(axis).get_pid_gains();
    return axes_pid_gains;
}

void PIDManager::run_loops(const AxesMap<double> &errors, const AxesMap<double> &deltaTimes, AxesMap<double> &outputs,
                           AxesMap<PIDInfo> &infos, const AxesMap<double> &derivatives)
{
    // Run PID loops with given errors and delta times
    // Store outputs in given outputs map
    for (const AxesEnum &axis : AXES)
    {
        double provided_derivative = (derivatives.count(axis) > 0) ? derivatives.at(axis) : 0;
        outputs[axis] = pid_controllers.at(axis).run_loop(errors.at(axis), deltaTimes.at(axis), infos[axis],
                                                          provided_derivative);
    }
}

void PIDManager::reset(AxesEnum axis_to_reset)
{
    // Reset PID loop at given axis
    pid_controllers.at(axis_to_reset).reset();
}