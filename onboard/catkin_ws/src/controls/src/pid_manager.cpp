#include "pid.h"
#include "pid_manager.h"
#include "controls_utils.h"

PIDManager::PIDManager()
{
    // Initialize PID controllers
    for (const AxisEnum &axis : AXES)
        pid_controllers[axis] = PID();
}

PIDManager::PIDManager(std::map<AxisEnum, std::map<PIDGainTypesEnum, double>> &pid_gains_for_axes)
{
    // Initialize PID controllers
    for (const AxisEnum &axis : AXES)
        pid_controllers[axis] = PID(pid_gains_for_axes[axis]);
}

void PIDManager::run_loops(std::map<AxisEnum, double> errors, std::map<AxisEnum, double> deltaTimes)
{
    // Run PID loops
    for (const AxisEnum &axis : AXES)
        pid_controllers[axis].run_loop(errors[axis], deltaTimes[axis]);
}