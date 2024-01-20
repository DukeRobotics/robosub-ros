#include <unordered_map>
#include "drc_pid.h"
#include "pid_manager.h"
#include "controls_utils.h"

PIDManager::PIDManager()
{
    // Initialize PID controllers with gains and terms as nullptr
    for (const AxesEnum &axis : AXES)
        pid_controllers[axis] = PID();
}

PIDManager::PIDManager(AxesPIDGainsMap pid_gains_for_axes, AxesPIDGainsMap pid_terms_for_axes)
{
    // Initialize PID controllers with given gains and terms pointers
    for (const AxesEnum &axis : AXES)
        pid_controllers[axis] = PID(pid_gains_for_axes[axis], pid_terms_for_axes[axis]);
}

void PIDManager::run_loops(const std::unordered_map<AxesEnum, double> &errors,
                           const std::unordered_map<AxesEnum, double> &deltaTimes,
                           std::unordered_map<AxesEnum, double> &outputs)
{
    // Run PID loops with given errors and delta times
    // Store outputs in given outputs map
    for (const AxesEnum &axis : AXES)
        outputs[axis] = pid_controllers.at(axis).run_loop(errors.at(axis), deltaTimes.at(axis));
}

void PIDManager::reset(AxesEnum axis_to_reset)
{
    // Reset PID loop at given axis
    pid_controllers.at(axis_to_reset).reset();
}