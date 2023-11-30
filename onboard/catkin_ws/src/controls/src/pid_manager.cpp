#include <unordered_map>
#include "pid.h"
#include "pid_manager.h"
#include "controls_utils.h"

PIDManager::PIDManager()
{
    // Initialize PID controllers
    for (const AxesEnum &axis : AXES)
        pid_controllers[axis] = PID();
}

PIDManager::PIDManager(AxesPIDGainsMap pid_gains_for_axes)
{
    // Initialize PID controllers
    for (const AxesEnum &axis : AXES)
        pid_controllers[axis] = PID(pid_gains_for_axes[axis]);
}

void PIDManager::run_loops(const std::unordered_map<AxesEnum, double> &errors,
                           const std::unordered_map<AxesEnum, double> &deltaTimes,
                           std::unordered_map<AxesEnum, double> &outputs)
{
    // Run PID loops
    for (const AxesEnum &axis : AXES)
        outputs[axis] = pid_controllers.at(axis).run_loop(errors.at(axis), deltaTimes.at(axis));
}