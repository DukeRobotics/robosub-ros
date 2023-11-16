#ifndef PID_MANAGER_H
#define PID_MANAGER_H

#include <map>
#include "controls_utils.h"
#include "pid.h"

class PIDManager
{
    public:
        std::map<AxisEnum, PID> pid_controllers;

        PIDManager();
        PIDManager(std::map<AxisEnum, std::map<PIDGainTypesEnum, double>> &pid_gains_for_axes);
        void run_loops(std::map<AxisEnum, double> errors, std::map<AxisEnum, double> deltaTimes);
};

#endif