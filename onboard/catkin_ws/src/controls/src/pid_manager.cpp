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

void PIDManager::run_loops(std::unordered_map<AxesEnum, double> errors, std::unordered_map<AxesEnum, double> deltaTimes)
{
    // Run PID loops
    for (const AxesEnum &axis : AXES)
        pid_controllers[axis].run_loop(errors[axis], deltaTimes[axis]);
}