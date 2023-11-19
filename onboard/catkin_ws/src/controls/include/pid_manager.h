#ifndef PID_MANAGER_H
#define PID_MANAGER_H

#include <memory>
#include <unordered_map>
#include "controls_utils.h"
#include "pid.h"

class PIDManager
{
    public:
        std::unordered_map<AxesEnum, PID> pid_controllers;

        PIDManager();
        PIDManager(AxesPIDGainsMap pid_gains_for_axes);
        void run_loops(std::unordered_map<AxesEnum, double> errors, std::unordered_map<AxesEnum, double> deltaTimes);
};

#endif